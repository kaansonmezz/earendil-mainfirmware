/* I2C bus recovery: unstick a locked bus where a slave holds SDA low.
 * Two-stage recovery:
 *   1. 9-clock pulse on SCL to force the slave to release SDA
 *   2. Full peripheral reset (RCC force/release) + HAL re-init
 *
 * Requires knowledge of the I2C GPIO pins.  For I2C1 on this board:
 *   SCL = PB8,  SDA = PB9,  both AF4 alternate function. */

#include "i2c_recovery.h"
#include "logger.h"

/* ── I2C1 pin definitions (PB8=SCL, PB9=SDA) ──────────────────────────── */
#define I2C_RECOVERY_SCL_PORT   GPIOB
#define I2C_RECOVERY_SCL_PIN    GPIO_PIN_8
#define I2C_RECOVERY_SDA_PORT   GPIOB
#define I2C_RECOVERY_SDA_PIN    GPIO_PIN_9

/* Number of SCL clock pulses to clock out during recovery. */
#define I2C_RECOVERY_CLK_PULSES  9U

/* Small delay for half-period of manual bit-bang (~5 us per toggle). */
#define I2C_RECOVERY_TOGGLE_US   5U

/* ── Helpers ───────────────────────────────────────────────────────────── */

static void delay_us(uint32_t us)
{
    /* Rough busy-loop: at 520 MHz SYSCLK this is very approximate.
     * Only used during recovery — not timing-critical. */
    for (uint32_t i = 0; i < us * 10; i++)
        __NOP();
}

static GPIO_PinState read_sda(void)
{
    return HAL_GPIO_ReadPin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN);
}

static void toggle_scl(void)
{
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_RESET);
    delay_us(I2C_RECOVERY_TOGGLE_US);
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_TOGGLE_US);
}

/* ── Stage 1: 9-clock pulse recovery ──────────────────────────────────── */

static HAL_StatusTypeDef clock_recovery(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;

    /* Enable GPIOB clock (should already be enabled, but be safe). */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure SCL as open-drain output (temporary, for bit-banging). */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C_RECOVERY_SCL_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_RECOVERY_SCL_PORT, &gpio);

    /* Configure SDA as input with pull-up (so we can read it). */
    GPIO_InitTypeDef sda_gpio = {0};
    sda_gpio.Pin   = I2C_RECOVERY_SDA_PIN;
    sda_gpio.Mode  = GPIO_MODE_INPUT;
    sda_gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_RECOVERY_SDA_PORT, &sda_gpio);

    /* Send I2C STOP condition first: SDA low->high while SCL is high. */
    HAL_GPIO_WritePin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_RESET);
    delay_us(I2C_RECOVERY_TOGGLE_US);
    HAL_GPIO_WritePin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_TOGGLE_US);
    HAL_GPIO_WritePin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_SET);
    delay_us(I2C_RECOVERY_TOGGLE_US);

    /* Clock out 9 pulses on SCL to let the slave release SDA. */
    for (uint32_t i = 0; i < I2C_RECOVERY_CLK_PULSES; i++)
        toggle_scl();

    /* Check if SDA is released (high). */
    GPIO_PinState sda_state = read_sda();

    /* Restore SCL to alternate function (I2C open-drain). */
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C_RECOVERY_SCL_PORT, &gpio);

    /* Restore SDA to alternate function. */
    sda_gpio.Mode  = GPIO_MODE_AF_OD;
    sda_gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C_RECOVERY_SDA_PORT, &sda_gpio);

    if (sda_state == GPIO_PIN_SET)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,CLOCK_PULSE,SDA:HIGH,OK:1");
        return HAL_OK;
    }

    Logger_Log(LOG_INFO, "I2C_RECOVERY,CLOCK_PULSE,SDA:LOW,OK:0");
    return HAL_ERROR;
}

/* ── Stage 2: Full peripheral reset ───────────────────────────────────── */

HAL_StatusTypeDef I2C_BusResetAndReinit(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_RESET,START");

    /* Force-reset the I2C1 peripheral via RCC. */
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_Delay(2);

    /* HAL_I2C_DeInit clears handle state. */
    HAL_I2C_DeInit(hi2c);

    /* Re-init I2C1 peripheral (calls HAL_I2C_MspInit internally for pins/clock). */
    /* We need to reconfigure the same parameters as MX_I2C1_Init. */
    hi2c->Instance = I2C1;
    hi2c->Init.Timing = 0x20A0ACFE;
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
        Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_RESET,INIT_FAIL,HAL:%d", (int)st);
        return HAL_ERROR;
    }

    HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);

    Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_RESET,DONE,HAL:0");
    return HAL_OK;
}

/* ── Stage 3: check if bus is idle (both SCL and SDA high) ────────────── */

static HAL_StatusTypeDef bus_idle_check(void)
{
    /* Quick check: if both lines are high the bus is free. */
    GPIO_PinState scl = HAL_GPIO_ReadPin(I2C_RECOVERY_SCL_PORT, I2C_RECOVERY_SCL_PIN);
    GPIO_PinState sda = HAL_GPIO_ReadPin(I2C_RECOVERY_SDA_PORT, I2C_RECOVERY_SDA_PIN);

    if (scl == GPIO_PIN_SET && sda == GPIO_PIN_SET)
        return HAL_OK;

    return HAL_ERROR;
}

/* ── Public API ────────────────────────────────────────────────────────── */

HAL_StatusTypeDef I2C_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "I2C_RECOVERY,START");

    /* If bus is already idle, nothing to do. */
    if (bus_idle_check() == HAL_OK)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,BUS_IDLE,NO_ACTION");
        return HAL_OK;
    }

    /* Try 9-clock pulse recovery first. */
    HAL_StatusTypeDef st = clock_recovery(hi2c);
    if (st == HAL_OK)
    {
        /* Re-init the peripheral after GPIO bit-banging. */
        st = I2C_BusResetAndReinit(hi2c);
        if (st == HAL_OK)
        {
            Logger_Log(LOG_INFO, "I2C_RECOVERY,CLOCK_OK");
            return HAL_OK;
        }
    }

    /* Clock recovery failed — do full peripheral reset. */
    Logger_Log(LOG_INFO, "I2C_RECOVERY,CLOCK_FAILED,TRYING_PERIPH_RESET");
    st = I2C_BusResetAndReinit(hi2c);
    if (st == HAL_OK)
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,PERIPH_RESET_OK");
        return HAL_OK;
    }

    Logger_Log(LOG_INFO, "I2C_RECOVERY,FAILED");
    return HAL_ERROR;
}

HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    return I2C_BusRecovery(hi2c);
}
