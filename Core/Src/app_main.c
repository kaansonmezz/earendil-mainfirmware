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
#include "motor_tuning_config.h"
#include "manipulation_uart_dma.h"
#include "i2c_scanner.h"
#include "imu_mpu9250.h"
#include "mag_qmc5883p.h"
#include "stm32h7xx_hal.h"

/* ── Private state ────────────────────────────────────────────────────────── */
static TerminalCommand_t s_parsedCmd;

#define TERMINAL_MAX_LINES_PER_UPDATE 16U
static char s_termLine[TERMINAL_RX_BUF_SIZE];

/* ── Public functions ─────────────────────────────────────────────────────── */

void App_Init(void)
{
    Logger_Init();
    I2C_ScanBus();

    /* Auto-init magnetometer (QMC5883L) at startup */
    {
        extern I2C_HandleTypeDef hi2c1;
        HAL_StatusTypeDef mag_st = MAG_QMC5883P_Init(&hi2c1, &g_mag_handle);
        if (mag_st != HAL_OK)
            Logger_Log(LOG_BOOT, "QMC5883P magnetometer init FAILED (will retry on read)");
    }

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

    ManipulationUartDma_Init();
    ManipulationUartDma_StartRx();

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

        if (TerminalParser_Parse(s_termLine, &s_parsedCmd))
        {
            /* Suppress CMD log for heartbeat — keep the serial link quiet. */
            if (s_parsedCmd.type != TCMD_HB)
                Logger_Log(LOG_INFO, "CMD: %s", s_termLine);

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
    ManipulationUartDma_Update();

    /* ── Periodic IMU stream (non-blocking) ───────────────────────── */
    IMU_StreamTask();

    /* NOTE: DISARM is a logical safety lock only — the CPU is never put into
     * WFI/STOP/STANDBY.  The main loop always runs at full speed so SWD debug
     * and ST-LINK flash/upload stay stable.  Motion is gated by
     * OperatingMode_IsDisarm() in command_handler.c and motor_dispatcher.c. */
}
