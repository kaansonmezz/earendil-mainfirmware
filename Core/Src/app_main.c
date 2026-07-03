#include "app_main.h"
#include "app_config.h"
#include "logger.h"
#include "terminal_if.h"
#include "terminal_parser.h"
#include "command_handler.h"
#include "control_mode.h"
#include "activity_light.h"
#include "operating_mode.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "ack_manager.h"
#include "safety_manager.h"
#include "motor_uart_dma.h"
#include "motor_tx_dma.h"
#include "i2c_scanner.h"
#include "imu_mpu9250.h"
#include "stm32h7xx_hal.h"

/* ── Private state ────────────────────────────────────────────────────────── */
static TerminalCommand_t s_parsedCmd;
static uint32_t s_imuLastTick = 0U;

#define TERMINAL_MAX_LINES_PER_UPDATE 16U
#define IMU_READ_INTERVAL_MS          100U
static char s_termLine[TERMINAL_RX_BUF_SIZE];

/* ── Public functions ─────────────────────────────────────────────────────── */

void App_Init(void)
{
    Logger_Init();
    I2C_ScanBus();
    TerminalIf_Init();

    ControlMode_Init();
    ActivityLight_Init();
    OperatingMode_Init();   /* starts in DISARM (hard safety lock) */

    MotionController_Init();
    MotorDispatcher_Init();
    MotorTxDma_Init();
    AckManager_Init();
    SafetyManager_Init();

    MotorUartDma_Init();
    MotorUartDma_StartAllRx();

    Logger_Log(LOG_BOOT, "H723 rover main controller started");
    Logger_Log(LOG_BOOT, "Operating mode: DISARM (motion locked)");
    Logger_Log(LOG_BOOT, "Default control mode: RPM");
    Logger_Log(LOG_BOOT, "Type 'help' for commands");
}

void App_Update(void)
{
    /* ── Terminal command processing ───────────────────────────────────
     * Drain all queued terminal lines each loop iteration so a GUI burst
     * (e.g. 7 tuning commands) is fully processed in order without loss. */
    uint8_t processed = 0U;
    while (processed < TERMINAL_MAX_LINES_PER_UPDATE &&
           TerminalIf_GetLine(s_termLine, sizeof(s_termLine)))
    {
        processed++;

        Logger_Log(LOG_INFO, "CMD: %s", s_termLine);

        if (TerminalParser_Parse(s_termLine, &s_parsedCmd))
        {
            CommandHandler_Handle(&s_parsedCmd);
        }
        else
        {
            Logger_Log(LOG_ERROR, "Unknown command: %s", s_termLine);
        }
    }

    /* ── Periodic module updates ─────────────────────────────────────── */
    MotorDispatcher_Update();
    AckManager_Update();
    SafetyManager_Update();
    MotorUartDma_Update();

    /* ── Periodic IMU converted read (10 Hz) ────────────────────────── */
    {
        uint32_t now = HAL_GetTick();
        if ((now - s_imuLastTick) >= IMU_READ_INTERVAL_MS)
        {
            s_imuLastTick = now;
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_Conv_t conv;
            HAL_StatusTypeDef st = IMU_MPU9250_ReadConverted(&hi2c1, &conv);
            uint8_t ok = (st == HAL_OK) ? 1U : 0U;
            if (ok)
            {
                Logger_Log(LOG_INFO,
                           "MPU_IMU,"
                           "AX:%ld,AY:%ld,AZ:%ld,"
                           "GX:%ld,GY:%ld,GZ:%ld,"
                           "TC:%ld,OK:1",
                           (long)conv.acc_x_mg, (long)conv.acc_y_mg, (long)conv.acc_z_mg,
                           (long)conv.gyro_x_mdps, (long)conv.gyro_y_mdps, (long)conv.gyro_z_mdps,
                           (long)conv.temp_cx100);
            }
            else
            {
                Logger_Log(LOG_INFO, "MPU_IMU,AX:0,AY:0,AZ:0,GX:0,GY:0,GZ:0,TC:0,OK:0");
            }
        }
    }

    /* NOTE: DISARM is a logical safety lock only — the CPU is never put into
     * WFI/STOP/STANDBY.  The main loop always runs at full speed so SWD debug
     * and ST-LINK flash/upload stay stable.  Motion is gated by
     * OperatingMode_IsDisarm() in command_handler.c and motor_dispatcher.c. */
}
