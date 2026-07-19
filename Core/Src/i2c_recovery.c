/* I2C bus recovery for STM32H7 — fault-tolerant implementation.
 *
 * Handles:
 *   A. Physical bus stuck (SDA low or SCL low) -> GPIO bit-bang recovery
 *   B. Lines idle but HAL/peripheral state stuck -> peripheral reset only
 *   C. Magnetometer internal state (handled by mag driver, not here)
 *
 * Shared I2C1: affects both QMC5883L and MPU9250.
 * Recovery is guarded against nested/recursive calls. */

#include "i2c_recovery.h"
#include "app_config.h"    /* I2C_TIMING_APP */
#include "logger.h"

extern I2C_HandleTypeDef hi2c1;

/* ── I2C1 pin definitions (PB8=SCL, PB9=SDA) ──────────────────────────── */
#define I2C_RECOVERY_SCL_PORT   GPIOB
#define I2C_RECOVERY_SCL_PIN    GPIO_PIN_8
#define I2C_RECOVERY_SDA_PORT   GPIOB
#define I2C_RECOVERY_SDA_PIN    GPIO_PIN_9

/* Number of SCL clock pulses maximum during recovery. */
#define I2C_RECOVERY_MAX_CLK_PULSES  9U

/* Recovery clock timing: ~10 us per half-period -> ~50 kHz recovery clock.
 * At 130 MHz I2C kernel clock, 10 us = 1,300,000 cycles. */
#define I2C_RECOVERY_HALF_PERIOD_US  10U

/* Timeout waiting for SCL to go high (another device may hold it). */
#define I2C_RECOVERY_SCL_WAIT_US     1000U

/* ── Recovery-in-progress guard ───────────────────────────────────────────── */
static volatile uint8_t s_recovery_in_progress = 0;

uint8_t I2C_RecoveryInProgress(void)
{
    return s_recovery_in_progress;
}

/* ── DWT-based microsecond delay ────────────────────────────────────────────
 * Uses the DWT cycle counter for accurate timing on Cortex-M7.
 * Falls back to a calibrated NOP loop if DWT is not available. */

static uint8_t s_dwt_available = 0;

static void DWT_Init(void)
{
    /* Enable DWT cycle counter if not already enabled */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Reset and enable the cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Verify it's running */
    uint32_t c1 = DWT->CYCCNT;
    __NOP();
    __NOP();
    __NOP();
    uint32_t c2 = DWT->CYCCNT;

    s_dwt_available = (c2 > c1) ? 1U : 0U;
}

static void delay_us(uint32_t us)
{
    if (s_dwt_available)
    {
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = us * (SystemCoreClock / 1000000U);
        while ((DWT->CYCCNT - start) < ticks)
            __NOP();
    }
    else
    {
        /* Fallback: calibrated for ~520 MHz SYSCLK.
         * Each iteration ~= 4 cycles (load, compare, NOP, branch).
         * 520 MHz / 4 = 130 iterations per microsecond. */
        volatile uint32_t count = us * (SystemCoreClock / 4000000U);
        while (count-- > 0)
            __NOP();
    }
}

/* ── GPIO helpers ────────────────────────────────────────────────────────── */

static GPIO_PinState ReadSDA(void)
{
    return HAL_GPIO_ReadPin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN);
}

static GPIO_PinState ReadSCL(void)
{
    return HAL_GPIO_ReadPin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN);
}

/* Configure SCL as open-drain output (for bit-banging). */
static void SCL_GPIO_Output(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C_RECOVERY_SCL_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_RECOVERY_SCL_PORT, &gpio);
}

/* Configure SDA as open-drain output (for driving STOP condition). */
static void SDA_GPIO_Output(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C_RECOVERY_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_RECOVERY_SDA_PORT, &gpio);
}

/* Configure SDA as input with pull-up (for reading). */
static void SDA_GPIO_Input(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C_RECOVERY_SDA_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_RECOVERY_SDA_PORT, &gpio);
}

/* Configure SCL as input with pull-up (for reading). */
static void SCL_GPIO_Input(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C_RECOVERY_SCL_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_RECOVERY_SCL_PORT, &gpio);
}

/* Restore both pins to I2C1 alternate function open-drain. */
static void RestoreI2C_AF(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = I2C_RECOVERY_SCL_PIN;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C_RECOVERY_SCL_PORT, &gpio);

    gpio.Pin       = I2C_RECOVERY_SDA_PIN;
    HAL_GPIO_Init(I2C_RECOVERY_SDA_PORT, &gpio);
}

/* ── Physical bus-clear sequence ──────────────────────────────────────────── */

static HAL_StatusTypeDef PhysicalBusClear(void)
{
    uint32_t clocks_sent = 0;
    uint8_t sda_released = 0;

    /* Ensure GPIOB clock is enabled */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 1. Disable the I2C peripheral so it no longer owns the pins.
     *    We must do this BEFORE bit-banging, otherwise the peripheral's
     *    AF output driver conflicts with our GPIO toggling. */
    if (hi2c1.Instance != NULL && (hi2c1.Instance->CR1 & I2C_CR1_PE))
    {
        hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
    }

    /* 2. Configure SCL as open-drain output, SDA as input with pull-up. */
    SCL_GPIO_Output();
    SDA_GPIO_Input();

    /* 3. Release SCL high first. */
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_HALF_PERIOD_US);

    /* 4. Read actual line states for diagnostics. */
    GPIO_PinState sda_init = ReadSDA();
    GPIO_PinState scl_init = ReadSCL();

    Logger_Log(LOG_INFO, "I2C_RECOVERY,REASON:PHYSICAL_BUS,SDA:%u,SCL:%u",
               (sda_init == GPIO_PIN_SET) ? 1U : 0U,
               (scl_init == GPIO_PIN_SET) ? 1U : 0U);

    /* 5. If SDA is already high, no bit-banging needed — just generate
     *    a STOP condition and proceed to peripheral reset. */
    if (sda_init == GPIO_PIN_SET)
    {
        goto generate_stop;
    }

    /* 6. SDA is low — generate up to 9 SCL pulses, checking SDA after each. */
    for (uint32_t i = 0; i < I2C_RECOVERY_MAX_CLK_PULSES; i++)
    {
        /* SCL low */
        HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_RESET);
        delay_us(I2C_RECOVERY_HALF_PERIOD_US);

        /* SCL high */
        HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);

        /* Wait for SCL to actually go high (timeout if held low by slave) */
        {
            uint32_t wait = 0;
            while (ReadSCL() == GPIO_PIN_RESET && wait < I2C_RECOVERY_SCL_WAIT_US)
            {
                delay_us(1);
                wait++;
            }
            if (wait >= I2C_RECOVERY_SCL_WAIT_US)
            {
                Logger_Log(LOG_INFO, "I2C_RECOVERY,SCL_HELD_LOW,PULSE:%lu",
                           (unsigned long)i);
            }
        }

        delay_us(I2C_RECOVERY_HALF_PERIOD_US);
        clocks_sent++;

        /* Check if SDA has been released */
        if (ReadSDA() == GPIO_PIN_SET)
        {
            sda_released = 1;
            break;
        }
    }

    Logger_Log(LOG_INFO, "I2C_RECOVERY,CLOCKS:%lu,SDA_RELEASED:%u",
               (unsigned long)clocks_sent, sda_released ? 1U : 0U);

generate_stop:
    /* 7. Generate a real STOP condition:
     *    - Drive SDA low while SCL is low
     *    - Release SCL high (verify it goes high)
     *    - Release SDA high (verify it goes high)
     */
    SDA_GPIO_Output();

    /* SDA low */
    HAL_GPIO_WritePin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_RESET);
    delay_us(I2C_RECOVERY_HALF_PERIOD_US);

    /* SCL low */
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_RESET);
    delay_us(I2C_RECOVERY_HALF_PERIOD_US);

    /* Release SCL high */
    SCL_GPIO_Input();
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_HALF_PERIOD_US);

    /* Release SDA high */
    SDA_GPIO_Input();
    HAL_GPIO_WritePin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_HALF_PERIOD_US);

    /* 8. Verify final line states */
    GPIO_PinState sda_final = ReadSDA();
    GPIO_PinState scl_final = ReadSCL();

    Logger_Log(LOG_INFO, "I2C_RECOVERY,FINAL_LINES,SDA:%u,SCL:%u",
               (sda_final == GPIO_PIN_SET) ? 1U : 0U,
               (scl_final == GPIO_PIN_SET) ? 1U : 0U);

    /* 9. Restore pins to I2C alternate function */
    RestoreI2C_AF();

    return (sda_final == GPIO_PIN_SET) ? HAL_OK : HAL_ERROR;
}

/* ── Peripheral reset + reinit ───────────────────────────────────────────── */

HAL_StatusTypeDef I2C_BusResetAndReinit(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_REINIT,START");

    /* Force-reset the I2C1 peripheral via RCC. */
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_Delay(2);

    /* DeInit clears handle state. */
    HAL_I2C_DeInit(hi2c);

    /* Re-init with same parameters as MX_I2C1_Init. */
    hi2c->Instance = I2C1;
    hi2c->Init.Timing = I2C_TIMING_APP;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_StatusTypeDef st = HAL_I2C_Init(hi2c);
    if (st != HAL_OK)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_REINIT,INIT_FAIL,HAL:%d", (int)st);
        return HAL_ERROR;
    }

    HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);

    Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_REINIT,DONE,OK:1");
    return HAL_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

HAL_StatusTypeDef I2C_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    /* Guard against nested recovery */
    if (s_recovery_in_progress)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,SKIPPED:NESTED");
        return HAL_BUSY;
    }
    s_recovery_in_progress = 1;

    /* Initialize DWT for accurate delays */
    DWT_Init();

    Logger_Log(LOG_INFO, "I2C_RECOVERY,START");

    /* Read line states */
    GPIO_PinState sda = ReadSDA();
    GPIO_PinState scl = ReadSCL();

    Logger_Log(LOG_INFO, "I2C_RECOVERY,SDA:%u,SCL:%u",
               (sda == GPIO_PIN_SET) ? 1U : 0U,
               (scl == GPIO_PIN_SET) ? 1U : 0U);

    HAL_StatusTypeDef phys_result = HAL_OK;   /* physical bus-clear result */
    HAL_StatusTypeDef periph_result = HAL_OK;  /* peripheral reinit result */

    /* Case A: Physical bus stuck (SDA low or SCL low).
     * Need GPIO bit-bang recovery. */
    if (sda == GPIO_PIN_RESET || scl == GPIO_PIN_RESET)
    {
        phys_result = PhysicalBusClear();

        /* After physical bus clear, reinit the peripheral. */
        periph_result = I2C_BusResetAndReinit(hi2c);
    }
    /* Case B: Lines high but HAL/peripheral state may be stuck.
     * Still need to reinit the peripheral. */
    else
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,BUS_IDLE,PERIPH_REINIT");
        periph_result = I2C_BusResetAndReinit(hi2c);
    }

    /* Invalidate MPU9250 address cache after bus recovery */
    IMU_MPU9250_InvalidateCache();

    /* Final line state verification */
    GPIO_PinState sda_final = ReadSDA();
    GPIO_PinState scl_final = ReadSCL();

    Logger_Log(LOG_INFO, "I2C_RECOVERY,RESULT,PHYS:%d,PERIPH:%d,SDA:%u,SCL:%u",
               (int)phys_result, (int)periph_result,
               (sda_final == GPIO_PIN_SET) ? 1U : 0U,
               (scl_final == GPIO_PIN_SET) ? 1U : 0U);

    /* Final determination: recovery succeeds only if:
     * - Peripheral reinit succeeded
     * - Both lines are high (no stuck bus)
     * Physical bus clear failure is also a failure, but if lines ended up
     * high anyway (e.g. slave released), we consider it OK. */
    HAL_StatusTypeDef result = periph_result;
    if (sda_final != GPIO_PIN_SET || scl_final != GPIO_PIN_SET)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,FAIL:LINES_STUCK");
        result = HAL_ERROR;
    }

    Logger_Log(LOG_INFO, "I2C_RECOVERY,DONE,OK:%u", (result == HAL_OK) ? 1U : 0U);
    s_recovery_in_progress = 0;
    return result;
}

HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    return I2C_BusRecovery(hi2c);
}
