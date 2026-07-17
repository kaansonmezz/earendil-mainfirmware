#include "command_handler.h"
#include "control_mode.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "motor_tx_dma.h"
#include "activity_light.h"
#include "operating_mode.h"
#include "safety_manager.h"
#include "terminal_if.h"
#include "motor_tuning_config.h"
#include "manipulation_uart_dma.h"
#include "logger.h"
#include "i2c_scanner.h"
#include "imu_mpu9250.h"
#include "mag_qmc5883p.h"
#include <string.h>

/* ── Tunables ───────────────────────────────────────────────────────────────
 *  Bounded wait for the motor TX DMA path to drain during a synchronized
 *  control-mode switch.  This is a main-loop context call (never ISR), so a
 *  short blocking poll on the TX busy/pending flags is safe and bounded. */
#define MODE_SWITCH_TX_DRAIN_MS 100U

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/* Returns the short terminal prefix for a motion command ("f", "fd", ...). */
static const char *MotionPrefix(Direction_t dir, bool isDuty)
{
    switch (dir)
    {
        case DIR_FORWARD:  return isDuty ? "fd" : "f";
        case DIR_BACKWARD: return isDuty ? "bd" : "b";
        case DIR_RIGHT:    return isDuty ? "rd" : "r";
        case DIR_LEFT:     return isDuty ? "ld" : "l";
        default:           return "?";
    }
}

/* Direct-motor raw payloads that are safe in DISARM (queries / stop /
 * brake / the documented control-mode switches).  Used by the DISARM
 * gate to reject motion-causing raw payloads like `FL f100` while still
 * allowing `FL status`, `FL identify`, `FL stop`, `FL x`,
 * `FL mode speed`, `FL mode duty`.  The payload reaches here already
 * lowercased and trimmed by the parser, so a plain strcmp is enough. */
static bool IsSafeRawPayload(const char *p)
{
    if (p == NULL)
        return false;
    return (strcmp(p, "status")     == 0 ||
            strcmp(p, "identify")  == 0 ||
            strcmp(p, "stop")      == 0 ||
            strcmp(p, "x")         == 0 ||
            strcmp(p, "cfg")       == 0 ||
            strcmp(p, "mode speed")== 0 ||
            strcmp(p, "mode duty") == 0);
}

static const char *MotorTagName(MotorId_t id)
{
    switch (id)
    {
        case MOTOR_FL: return "FL";
        case MOTOR_FR: return "FR";
        case MOTOR_RL: return "RL";
        case MOTOR_RR: return "RR";
        default:        return "??";
    }
}

/* ── Command classification helpers ─────────────────────────────────────────
 *  Used by the DISARM gate to decide what is allowed while locked. */

bool Command_IsModeTransition(const TerminalCommand_t *cmd)
{
    return (cmd != NULL && cmd->type == TCMD_OP_MODE);
}

bool Command_IsMotionCommand(const TerminalCommand_t *cmd)
{
    return (cmd != NULL && (cmd->type == TCMD_MOTION ||
                            cmd->type == TCMD_DRIVE_ARC));
}

/* ── Operating-mode transition ──────────────────────────────────────────────
 *  Enforces the DISARM safety lock side-effects: on entering DISARM all
 *  motors are safe-zeroed and stale motion is neutralized; on leaving,
 *  motors stay stopped and a fresh command is required to move. */
static void HandleOperatingMode(RoverMode_t target)
{
    if (target == ROVER_MODE_DISARM)
    {
        OperatingMode_Set(ROVER_MODE_DISARM);
        SafetyManager_EnterDisarm();
        Logger_Log(LOG_INFO, "[MODE] DISARM active, motion commands locked");
        return;
    }

    /* Leaving DISARM -> MANUAL or AUTONOMOUS */
    OperatingMode_Set(target);
    SafetyManager_LeaveDisarm();

    if (target == ROVER_MODE_MANUAL)
        Logger_Log(LOG_INFO, "[MODE] MANUAL active");
    else if (target == ROVER_MODE_AUTONOMOUS)
        Logger_Log(LOG_INFO, "[MODE] AUTONOMOUS active");
    else
        Logger_Log(LOG_INFO, "[MODE] %s active", OperatingMode_ToString(target));

    Logger_Log(LOG_INFO, "Motors stopped; send a motion command to move");
}

/* ── Synchronized control-mode switch (RPM <-> PWM) ──────────────────────────
 *  The F411 motor controllers reject `mode speed` / `mode duty` while a motor
 *  is still running, replying e.g. "[ERR] Stop motor first".  That would leave
 *  H7 and F411 desynchronized (H7 believes it changed mode, F411 did not).
 *
 *  Safe policy implemented here:
 *    1. Announce the switch.
 *    2. Stop all motors first (`stop`), dropping any queued motion frame so
 *       the stop leaves immediately instead of being stalled behind a pending
 *       motion frame.
 *    3. Wait for the stop frame to fully drain on every motor UART (bounded
 *       poll on the TX DMA busy/pending flags).
 *    4. Send the requested `mode speed` / `mode duty` to all controllers.
 *    5. Only after every channel accepted the frame for TX dispatch, update
 *       the local control mode.
 *
 *  ACK confirmation from the F411 is NOT used: the existing ACK/OK parsing is
 *  not wired to raw commands (SendRaw does not register a pending ACK and the
 *  RX callback only logs replies), so we must not fake full confirmation.  The
 *  local mode is therefore advanced only after successful TX dispatch of the
 *  mode command, and the log explicitly states the change was dispatched but
 *  not fully ACK-confirmed. */
static bool WaitForTxDrain(uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();
    while (!MotorTxDma_AllIdle())
    {
        if ((HAL_GetTick() - start) >= timeoutMs)
            return false;
    }
    return true;
}

static void HandleControlModeSwitch(ControlMode_t target)
{
    const char *modeName = (target == CONTROL_MODE_RPM) ? "SPEED" : "DUTY";
    const char *modeCmd  = (target == CONTROL_MODE_RPM) ? "mode speed" : "mode duty";

    Logger_Log(LOG_INFO, "[MODE] Switching motor controllers to %s...", modeName);

    /* 1. Stop all motors first so the F411s accept the mode command.
     *    Cancel any queued (non-active) motion frame so `stop` leaves
     *    immediately rather than sitting behind a pending motion frame. */
    Logger_Log(LOG_INFO, "[MODE] Sending stop before mode change");
    MotorTxDma_CancelPending();
    MotorDispatcher_SendRaw("stop");

    /* 2. Let the stop frame fully drain on every channel before issuing the
     *    mode command—otherwise the mode command could be staged behind the
     *    stop and arrive too early (or be dropped by the pending-slot policy). */
    if (!WaitForTxDrain(MODE_SWITCH_TX_DRAIN_MS))
    {
        Logger_Log(LOG_ERROR,
                   "[MODE] Stop TX did not drain within %ums; mode switch aborted, "
                   "local control mode unchanged (%s)",
                   (unsigned)MODE_SWITCH_TX_DRAIN_MS,
                   ControlMode_ToString(ControlMode_Get()));
        return;
    }

    /* 3. Dispatch the mode command to all motor controllers. */
    if (!MotorDispatcher_SendRaw(modeCmd))
    {
        Logger_Log(LOG_ERROR,
                   "[MODE] Failed to queue '%s' to one or more motors; "
                   "local control mode unchanged (%s)",
                   modeCmd,
                   ControlMode_ToString(ControlMode_Get()));
        return;
    }
    Logger_Log(LOG_INFO, "[MODE] Sent %s to all motors", modeCmd);

    /* 4. Only now advance the local control mode — after successful TX
     *    dispatch of the mode command.  Subsequent motion commands (e.g.
     *    f100 in RPM mode) will then be encoded as `rpm 100` / `rpm -100`. */
    ControlMode_Set(target);
    Logger_Log(LOG_INFO, "[MODE] Local control mode set to %s", modeName);
    Logger_Log(LOG_INFO,
               "[MODE] Mode command dispatched, not fully ACK-confirmed "
               "(F411 ACK parsing not wired for raw commands)");
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void CommandHandler_PrintHelp(void)
{
    Logger_Log(LOG_INFO, "Available commands:");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Control mode:");
    Logger_Log(LOG_INFO, "  m speed         Set RPM mode and forward \"mode speed\"");
    Logger_Log(LOG_INFO, "  m duty          Set PWM/duty mode and forward \"mode duty\"");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Rover mode (operating mode):");
    Logger_Log(LOG_INFO, "  mode             Show current rover mode (disarm/manual/auto)");
    Logger_Log(LOG_INFO, "  mode disarm      Disarm rover (red LED, STOP motors, lock motion)");
    Logger_Log(LOG_INFO, "  mode manual      Manual mode (green LED, motors stopped)");
    Logger_Log(LOG_INFO, "  mode auto        Autonomous mode (yellow LED, motors stopped)");
    Logger_Log(LOG_INFO, "  mode autonomous  Alias for 'mode auto'");
    Logger_Log(LOG_INFO, "  While DISARM: only mode/status/help/stop/brake/tuning are accepted.");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "RPM mode commands:");
    Logger_Log(LOG_INFO, "  f0..f200         Forward RPM command");
    Logger_Log(LOG_INFO, "  b0..b200         Backward RPM command");
    Logger_Log(LOG_INFO, "  r0..r200         Right turn RPM command");
    Logger_Log(LOG_INFO, "  l0..l200         Left turn RPM command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "PWM mode commands:");
    Logger_Log(LOG_INFO, "  fd0..fd4000      Forward PWM/duty command");
    Logger_Log(LOG_INFO, "  bd0..bd4000      Backward PWM/duty command");
    Logger_Log(LOG_INFO, "  rd0..rd4000      Right turn PWM/duty command");
    Logger_Log(LOG_INFO, "  ld0..ld4000      Left turn PWM/duty command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Arc-turn drive commands:");
    Logger_Log(LOG_INFO, "  drive rpm 0..200 <fl|fr|bl|br> tr 0.00..1.00");
    Logger_Log(LOG_INFO, "  drive duty 0..4000 <fl|fr|bl|br> tr 0.00..1.00");
    Logger_Log(LOG_INFO, "Examples:");
    Logger_Log(LOG_INFO, "  drive rpm 100 fl tr 0.50");
    Logger_Log(LOG_INFO, "  drive rpm 100 fr tr 0.50");
    Logger_Log(LOG_INFO, "  drive duty 2000 bl tr 0.50");
    Logger_Log(LOG_INFO, "  drive duty 2000 br tr 0.50");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Common commands:");
    Logger_Log(LOG_INFO, "  stop             Stop motors");
    Logger_Log(LOG_INFO, "  brake            Send brake command: x");
    Logger_Log(LOG_INFO, "  identify         Arm motors, then send identify to all motor UARTs");
    Logger_Log(LOG_INFO, "  status           Send status to all motor UARTs");
    Logger_Log(LOG_INFO, "  hb               Heartbeat / control-link keepalive (silent)");
    Logger_Log(LOG_INFO, "  linkstat         Diagnostic control-link status");
    Logger_Log(LOG_INFO, "  termstat         Terminal RX queue diagnostics");
    Logger_Log(LOG_INFO, "  i2cscan          Scan I2C1 bus for devices");
    Logger_Log(LOG_INFO, "  mpuwho           Read MPU9250 WHO_AM_I register");
    Logger_Log(LOG_INFO, "  mpuregs          Read MPU9250 diagnostic registers");
    Logger_Log(LOG_INFO, "  mpuwarm          Probe before/after I2C warm-up only");
    Logger_Log(LOG_INFO, "  mpuinit          Basic MPU6500/9250 init (reset, clock, accel/gyro)");
    Logger_Log(LOG_INFO, "  mpucfgtest       CONFIG register write/readback diagnostic");
    Logger_Log(LOG_INFO, "  mpuraw           One-shot raw accel/gyro/temperature read");
    Logger_Log(LOG_INFO, "  mpudbgraw        Update IMU raw debug variables for CubeIDE");
    Logger_Log(LOG_INFO, "  mpugyrotest      Diagnose gyro raw registers and gyro enable state");
    Logger_Log(LOG_INFO, "  mpuconv           Read MPU accel/gyro/temp converted units");
    Logger_Log(LOG_INFO, "  mpubias           Query gyro static bias state");
    Logger_Log(LOG_INFO, "  mpubiason         Enable gyro bias correction");
    Logger_Log(LOG_INFO, "  mpubiasoff        Disable gyro bias correction");
    Logger_Log(LOG_INFO, "  mpubiasclear      Clear gyro bias to zero");
    Logger_Log(LOG_INFO, "  imu help           Show IMU command list");
    Logger_Log(LOG_INFO, "  imu stream on      Enable periodic IMU telemetry");
    Logger_Log(LOG_INFO, "  imu stream off     Disable periodic IMU telemetry");
    Logger_Log(LOG_INFO, "  imu telper <ms>    Set IMU telemetry period (20..5000)");
    Logger_Log(LOG_INFO, "  imu gyrofilter status  Show gyro output filter settings");
    Logger_Log(LOG_INFO, "  imu gyrofilter on      Enable gyro output filter");
    Logger_Log(LOG_INFO, "  imu gyrofilter off     Disable gyro output filter");
    Logger_Log(LOG_INFO, "  imu deadband <mdps>    Set gyro display deadband (0..2000)");
    Logger_Log(LOG_INFO, "  imu lpf <permille>     Set gyro EMA alpha (1..1000)");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Magnetometer commands:");
    Logger_Log(LOG_INFO, "  magwho              Detect QMC5883P magnetometer");
    Logger_Log(LOG_INFO, "  maginit             Initialize QMC5883P magnetometer");
    Logger_Log(LOG_INFO, "  magraw              Read raw magnetometer X/Y/Z");
    Logger_Log(LOG_INFO, "  magimu              Read compact GUI-friendly magnetometer X/Y/Z");
    Logger_Log(LOG_INFO, "  maghelp             Show magnetometer commands");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Direct motor command:");
    Logger_Log(LOG_INFO, "  FL <text>        Send raw text only to Front Left motor");
    Logger_Log(LOG_INFO, "  FR <text>        Send raw text only to Front Right motor");
    Logger_Log(LOG_INFO, "  RL <text>        Send raw text only to Rear Left motor");
    Logger_Log(LOG_INFO, "  RR <text>        Send raw text only to Rear Right motor");
    Logger_Log(LOG_INFO, "Examples:");
    Logger_Log(LOG_INFO, "  FL status");
    Logger_Log(LOG_INFO, "  FR identify");
    Logger_Log(LOG_INFO, "  RL f100");
    Logger_Log(LOG_INFO, "  RR mode speed");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Manipulation arm command:");
    Logger_Log(LOG_INFO, "  arm <payload>    Send raw payload to manipulation F411 over UART8");
    Logger_Log(LOG_INFO, "Examples:");
    Logger_Log(LOG_INFO, "  arm forward 1 200");
    Logger_Log(LOG_INFO, "  arm set 1 stopmode brake");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Motor tuning:");
    Logger_Log(LOG_INFO, "  FL base P1 P2 P3 P4 P5 P6 P7 P8");
    Logger_Log(LOG_INFO, "  FL boost P1 P2 P3 P4 P5 P6 P7 P8 MS");
    Logger_Log(LOG_INFO, "  FL kickduty VALUE    (or: FL kick duty VALUE)");
    Logger_Log(LOG_INFO, "  FL kickms VALUE      (or: FL kick ms VALUE)");
    Logger_Log(LOG_INFO, "  FL ramp UP DOWN");
    Logger_Log(LOG_INFO, "  FL pi KP KI");
    Logger_Log(LOG_INFO, "  FL telper MS");
    Logger_Log(LOG_INFO, "  ALL base / boost / kickduty / kickms / ramp / pi / telper");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Config cache:");
    Logger_Log(LOG_INFO, "  cfgcache            Print cached tuning config for all motors");
    Logger_Log(LOG_INFO, "  cfgcache FL         Print cached config for Front Left");
    Logger_Log(LOG_INFO, "  cfgcache FR         Print cached config for Front Right");
    Logger_Log(LOG_INFO, "  cfgcache RL         Print cached config for Rear Left");
    Logger_Log(LOG_INFO, "  cfgcache RR         Print cached config for Rear Right");
    Logger_Log(LOG_INFO, "  cfgread FL          Send 'FL cfg' and cache response");
    Logger_Log(LOG_INFO, "  cfgread all         Send cfg to all motors");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "  help             Show this command list");
}

void CommandHandler_Handle(const TerminalCommand_t *cmd)
{
    if (cmd == NULL)
        return;

    /* ── Primary DISARM safety gate ────────────────────────────────────
     * While DISARM is active, only mode transitions and harmless query/
     * stop/brake commands are accepted.  Every motion-causing or
     * control-changing command is rejected so the rover cannot move. */
    if (OperatingMode_IsDisarm())
    {
        bool allowed = false;
        switch (cmd->type)
        {
            case TCMD_OP_MODE:      /* mode disarm/manual/auto/autonomous */
            case TCMD_STOP:         /* stop — always safe, even in DISARM */
            case TCMD_BRAKE:        /* brake — always safe, even in DISARM */
            case TCMD_HELP:         /* help */
            case TCMD_STATUS:       /* status (query) */
            case TCMD_TERMSTAT:     /* termstat (query) */
            case TCMD_I2CSCAN:     /* i2cscan (query) */
            case TCMD_MPUWHO:     /* mpuwho (query) */
            case TCMD_MPUREGS:   /* mpuregs (query) */
            case TCMD_MPUWARM:  /* mpuwarm (query) */
            case TCMD_MPUINIT:  /* mpuinit (init) */
            case TCMD_MPUCFGTEST: /* mpucfgtest (diagnostic) */
            case TCMD_MPURAW: /* mpuraw (query) */
            case TCMD_MPUDDBGRAW: /* mpudbgraw (query) */
            case TCMD_MPUGYROTEST: /* mpugyrotest (diagnostic) */
            case TCMD_MPUCONV: /* mpuconv (query) */
            case TCMD_MPUBIAS: /* mpubias (query) */
            case TCMD_MPUBIASON: /* mpubiason (config) */
            case TCMD_MPUBIASOFF: /* mpubiasoff (config) */
            case TCMD_MPUBIASCLEAR: /* mpubiasclear (config) */
            case TCMD_IMU_HELP:      /* imu help (query) */
            case TCMD_IMU_STREAM_ON: /* imu stream on (config) */
            case TCMD_IMU_STREAM_OFF:/* imu stream off (config) */
            case TCMD_IMU_TELPER:    /* imu telper (config) */
            case TCMD_IMU_GYROFILTER_STATUS: /* imu gyrofilter status (query) */
            case TCMD_IMU_GYROFILTER_ON:     /* imu gyrofilter on (config) */
            case TCMD_IMU_GYROFILTER_OFF:    /* imu gyrofilter off (config) */
            case TCMD_IMU_DEADBAND:          /* imu deadband (config) */
            case TCMD_IMU_LPF:               /* imu lpf (config) */
            case TCMD_MAGWHO:                /* magwho (query) */
            case TCMD_MAGINIT:               /* maginit (init) */
            case TCMD_MAGRAW:                /* magraw (query) */
            case TCMD_MAGIMU:                /* magimu (query) */
            case TCMD_MAGHELP:               /* maghelp (query) */
            case TCMD_CFGCACHE:              /* cfgcache (query) */
            case TCMD_CFGREAD:               /* cfgread (query) */
            case TCMD_HB:                    /* hb/heartbeat (keepalive) */
            case TCMD_LINKSTAT:              /* linkstat (diagnostic) */
                allowed = true;
                break;

            case TCMD_MOTOR_RAW:
                /* Empty payload (bare "FL") -> usage error, emit here so
                 * the handler's main switch does not need a DISARM copy. */
                if (cmd->rawPayload[0] == '\0')
                {
                    Logger_Log(LOG_ERROR,
                               "Usage: FL <text> | FR <text> | "
                               "RL <text> | RR <text>");
                    return;
                }
                /* Otherwise only safe payloads may pass; motion-causing
                 * raw commands (e.g. FL f100) are blocked. */
                allowed = IsSafeRawPayload(cmd->rawPayload);
                if (!allowed)
                {
                    Logger_Log(LOG_WARN,
                               "[DISARM] Direct motor command blocked");
                    return;
                }
                break;

            case TCMD_DRIVE_ARC:
                Logger_Log(LOG_WARN, "[DRIVE] Command rejected in DISARM");
                return;

            default:
                allowed = false;
                break;
        }

        if (!allowed)
        {
            Logger_Log(LOG_WARN, "[DISARM] Command ignored. Change mode first.");
            return;
        }
    }

    /* ── Heartbeat: lightweight keepalive — no log, no ACK, no motor TX ── */
    if (cmd->type == TCMD_HB)
    {
        SafetyManager_NotifyPcActivity();
        return;
    }

    /* ── Linkstat: diagnostic one-liner ──────────────────────────────── */
    if (cmd->type == TCMD_LINKSTAT)
    {
        uint32_t age = SafetyManager_PcLinkAgeMs();
        Logger_Log(LOG_INFO,
                   "PC_LINK,SEEN:%u,ALIVE:%u,TIMEOUT:%u,AGE_MS:%lu,LIMIT_MS:%lu",
                   SafetyManager_IsPcLinkSeen()  ? 1U : 0U,
                   SafetyManager_IsPcLinkAlive() ? 1U : 0U,
                   SafetyManager_IsPcLinkTimeout() ? 1U : 0U,
                   (unsigned long)age,
                   (unsigned long)PC_LINK_TIMEOUT_MS);
        return;
    }

    /* ── PC-link motion gate ─────────────────────────────────────────────
     *  Motion and arc-turn commands are rejected when the PC link has
     *  timed out or has never been established.  stop/brake/mode/status
     *  and all diagnostic/query commands pass through unconditionally. */
    if (cmd->type == TCMD_MOTION || cmd->type == TCMD_DRIVE_ARC)
    {
        if (!SafetyManager_IsPcLinkAlive())
        {
            Logger_Log(LOG_ERROR,
                       "[PC_LINK] Motion rejected: control link unavailable");
            return;
        }
    }

    /* ── PC-link mode gate ──────────────────────────────────────────────
     *  MANUAL and AUTONOMOUS require an active PC link.  DISARM is always
     *  accepted regardless of link state.  This prevents a stale mode
     *  command from reviving a dead link and enabling motion. */
    if (cmd->type == TCMD_OP_MODE)
    {
        if (cmd->opMode != ROVER_MODE_DISARM && !SafetyManager_IsPcLinkAlive())
        {
            Logger_Log(LOG_ERROR,
                       "[PC_LINK] Mode rejected: control link unavailable");
            return;
        }
    }

    /* ── Refresh PC-link activity on accepted control commands ──────────
     *  Only actual control actions refresh the watchdog.  Diagnostics,
     *  queries, IMU commands, motor config reads, and telemetry commands
     *  do NOT keep the control-link watchdog alive. */
    switch (cmd->type)
    {
        case TCMD_MOTION:
        case TCMD_DRIVE_ARC:
        case TCMD_STOP:
        case TCMD_BRAKE:
        case TCMD_OP_MODE:
            SafetyManager_NotifyPcActivity();
            break;
        default:
            break;
    }

    switch (cmd->type)
    {
        case TCMD_HELP:
            CommandHandler_PrintHelp();
            break;

        case TCMD_OP_MODE:
            HandleOperatingMode(cmd->opMode);
            break;

        case TCMD_STOP:
            MotionController_Execute(&cmd->motion);
            break;

        case TCMD_MOTION:
        {
            /* Report clamping first (was previously emitted by the parser). */
            if (cmd->wasClamped)
            {
                Logger_Log(LOG_WARN, "%s value %u clamped to %u",
                           MotionPrefix(cmd->motion.direction, cmd->isDuty),
                           cmd->originalValue, cmd->value);
            }

            ControlMode_t mode = ControlMode_Get();

            if (cmd->isDuty && mode != CONTROL_MODE_PWM)
            {
                Logger_Log(LOG_ERROR, "Invalid mode: duty commands require PWM mode");
                break;
            }

            if (!cmd->isDuty && mode != CONTROL_MODE_RPM)
            {
                Logger_Log(LOG_ERROR, "Invalid mode: RPM commands require RPM mode");
                break;
            }

            MotionController_Execute(&cmd->motion);
            break;
        }

        case TCMD_DRIVE_ARC:
        {
            uint16_t inner, outer;
            MotionController_ExecuteArcTurn(
                cmd->driveIsDuty, cmd->driveTarget,
                cmd->driveMotion, cmd->driveTurnRatioPermille,
                &inner, &outer);
            break;
        }

        case TCMD_BRAKE:
            MotorDispatcher_SendRaw("x");
            break;

        case TCMD_MODE_RPM:
            HandleControlModeSwitch(CONTROL_MODE_RPM);
            break;

        case TCMD_MODE_PWM:
            HandleControlModeSwitch(CONTROL_MODE_PWM);
            break;

        case TCMD_MODE_QUERY:
            Logger_Log(LOG_INFO, "Rover mode: %s",
                       OperatingMode_ToString(OperatingMode_Get()));
            break;

        case TCMD_IDENTIFY:
            /* Arm the motor controllers first (separate line), then
             * send the identify probe on its own line.  Waiting for the
             * TX DMA path to drain between the two guarantees ordering and
             * prevents the identify frame from being staged behind (or
             * dropped by) the pending-slot policy of the arm frame. */
            MotorDispatcher_SendRaw("arm service CURRENT_LIMITED_BENCH_SUPPLY");
            if (!WaitForTxDrain(MODE_SWITCH_TX_DRAIN_MS))
            {
                Logger_Log(LOG_ERROR,
                           "[IDENTIFY] Arm frame TX did not drain within %ums; "
                           "identify not sent",
                           (unsigned)MODE_SWITCH_TX_DRAIN_MS);
                break;
            }
            MotorDispatcher_SendRaw("identify");
            break;

        case TCMD_STATUS:
            MotorDispatcher_SendRaw("status");
            break;

        case TCMD_TERMSTAT:
            Logger_Log(LOG_INFO, "[TERM] rx_lines=%lu dropped=%lu pending=%u max_depth=%u",
                       (unsigned long)TerminalIf_GetReceivedLineCount(),
                       (unsigned long)TerminalIf_GetDroppedLineCount(),
                       (unsigned)TerminalIf_GetPendingLineCount(),
                       (unsigned)TerminalIf_GetMaxLineQueueDepth());
            break;

        case TCMD_I2CSCAN:
            I2C_ScanBus();
            break;

        case TCMD_MPUWHO:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_WhoAmI(&hi2c1);
            break;
        }

        case TCMD_MPUREGS:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_WhoAmI(&hi2c1);
            break;
        }

        case TCMD_MPUWARM:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_WarmupProbe(&hi2c1);
            break;
        }

        case TCMD_MPUINIT:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_InitBasic(&hi2c1);
            break;
        }

        case TCMD_MPUCFGTEST:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_CfgTest(&hi2c1);
            break;
        }

        case TCMD_MPURAW:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_Raw_t raw;
            HAL_StatusTypeDef st = IMU_MPU9250_ReadRaw(&hi2c1, &raw);
            uint8_t ok = (st == HAL_OK) ? 1U : 0U;
            IMU_MPU9250_UpdateDebugRaw(ok ? &raw : NULL, ok);
            Logger_Log(LOG_INFO,
                       "MPU_RAW,ACC_X:%d,ACC_Y:%d,ACC_Z:%d,"
                       "TEMP:%d,GYRO_X:%d,GYRO_Y:%d,GYRO_Z:%d,OK:%u",
                       (int)raw.acc_x, (int)raw.acc_y, (int)raw.acc_z,
                       (int)raw.temp,
                       (int)raw.gyro_x, (int)raw.gyro_y, (int)raw.gyro_z,
                       ok);
            break;
        }

        case TCMD_MPUDDBGRAW:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_Raw_t raw;
            HAL_StatusTypeDef st = IMU_MPU9250_ReadRaw(&hi2c1, &raw);
            uint8_t ok = (st == HAL_OK) ? 1U : 0U;
            IMU_MPU9250_UpdateDebugRaw(ok ? &raw : NULL, ok);
            Logger_Log(LOG_INFO,
                       "MPU_DBGRAW,ACC_X:%d,ACC_Y:%d,ACC_Z:%d,"
                       "TEMP:%d,GYRO_X:%d,GYRO_Y:%d,GYRO_Z:%d,OK:%u",
                       (int)raw.acc_x, (int)raw.acc_y, (int)raw.acc_z,
                       (int)raw.temp,
                       (int)raw.gyro_x, (int)raw.gyro_y, (int)raw.gyro_z,
                       ok);
            break;
        }

        case TCMD_MPUGYROTEST:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_GyroTest(&hi2c1);
            break;
        }

        case TCMD_MPUCONV:
        {
            extern I2C_HandleTypeDef hi2c1;
            IMU_MPU9250_Conv_t conv;
            HAL_StatusTypeDef st = IMU_MPU9250_ReadConverted(&hi2c1, &conv);
            uint8_t ok = (st == HAL_OK) ? 1U : 0U;

            int32_t gx = conv.gyro_x_mdps;
            int32_t gy = conv.gyro_y_mdps;
            int32_t gz = conv.gyro_z_mdps;
            IMU_ApplyGyroFilter(&gx, &gy, &gz);

            Logger_Log(LOG_INFO,
                       "MPU_CONV_MILLI,"
                       "ACC_X_MG:%ld,ACC_Y_MG:%ld,ACC_Z_MG:%ld,"
                       "TEMP_CX100:%ld,"
                       "GYRO_X_MDPS:%ld,GYRO_Y_MDPS:%ld,GYRO_Z_MDPS:%ld,"
                       "BIAS:%u,BSRC:%u,GFILT:%u,GDB:%ld,GLPF:%ld,OK:%u",
                       (long)conv.acc_x_mg, (long)conv.acc_y_mg, (long)conv.acc_z_mg,
                       (long)conv.temp_cx100,
                       (long)gx, (long)gy, (long)gz,
                       IMU_MPU9250_BiasIsEnabled(), IMU_MPU9250_BiasGetSource(),
                       IMU_GyroFilterIsEnabled(),
                       (long)IMU_GyroFilterGetDeadband(), (long)IMU_GyroFilterGetLpfAlpha(),
                       ok);
            Logger_Log(LOG_INFO,
                       "MPU_IMU,"
                       "AX:%ld,AY:%ld,AZ:%ld,"
                       "GX:%ld,GY:%ld,GZ:%ld,"
                       "TC:%ld,BIAS:%u,BSRC:%u,GFILT:%u,GDB:%ld,GLPF:%ld,OK:%u",
                       (long)conv.acc_x_mg, (long)conv.acc_y_mg, (long)conv.acc_z_mg,
                       (long)gx, (long)gy, (long)gz,
                       (long)conv.temp_cx100,
                       IMU_MPU9250_BiasIsEnabled(), IMU_MPU9250_BiasGetSource(),
                       IMU_GyroFilterIsEnabled(),
                       (long)IMU_GyroFilterGetDeadband(), (long)IMU_GyroFilterGetLpfAlpha(),
                       ok);
            break;
        }

        case TCMD_MPUBIAS:
        {
            IMU_MPU9250_BiasQuery();
            break;
        }

        case TCMD_MPUBIASON:
        {
            IMU_MPU9250_BiasEnable();
            break;
        }

        case TCMD_MPUBIASOFF:
        {
            IMU_MPU9250_BiasDisable();
            break;
        }

        case TCMD_MPUBIASCLEAR:
        {
            IMU_MPU9250_BiasClear();
            break;
        }

        case TCMD_IMU_HELP:
        {
            Logger_Log(LOG_INFO, "IMU_HELP,COMMANDS:");
            Logger_Log(LOG_INFO, "  imu help");
            Logger_Log(LOG_INFO, "  imu stream on");
            Logger_Log(LOG_INFO, "  imu stream off");
            Logger_Log(LOG_INFO, "  imu telper <ms>");
            Logger_Log(LOG_INFO, "  imu gyrofilter status");
            Logger_Log(LOG_INFO, "  imu gyrofilter on");
            Logger_Log(LOG_INFO, "  imu gyrofilter off");
            Logger_Log(LOG_INFO, "  imu deadband <mdps>");
            Logger_Log(LOG_INFO, "  imu lpf <alpha_permille>");
            Logger_Log(LOG_INFO, "  mpuwho");
            Logger_Log(LOG_INFO, "  mpuinit");
            Logger_Log(LOG_INFO, "  mpuraw");
            Logger_Log(LOG_INFO, "  mpuconv");
            Logger_Log(LOG_INFO, "  mpubias");
            Logger_Log(LOG_INFO, "  mpubiason");
            Logger_Log(LOG_INFO, "  mpubiasoff");
            Logger_Log(LOG_INFO, "  mpubiasclear");
            Logger_Log(LOG_INFO, "  mpucfgtest");
            Logger_Log(LOG_INFO, "  mpugyrotest");
            Logger_Log(LOG_INFO, "  magwho");
            Logger_Log(LOG_INFO, "  maginit");
            Logger_Log(LOG_INFO, "  magraw");
            Logger_Log(LOG_INFO, "  maghelp");
            break;
        }

        case TCMD_IMU_STREAM_ON:
        {
            IMU_StreamOn();
            break;
        }

        case TCMD_IMU_STREAM_OFF:
        {
            IMU_StreamOff();
            break;
        }

        case TCMD_IMU_TELPER:
        {
            if (!cmd->hasValue)
            {
                Logger_Log(LOG_INFO, "IMU_TELPER,ERR:MISSING_VALUE,OK:0");
                break;
            }
            IMU_StreamSetPeriod(cmd->value);
            break;
        }

        case TCMD_IMU_GYROFILTER_STATUS:
        {
            IMU_GyroFilterStatus();
            break;
        }

        case TCMD_IMU_GYROFILTER_ON:
        {
            IMU_GyroFilterOn();
            break;
        }

        case TCMD_IMU_GYROFILTER_OFF:
        {
            IMU_GyroFilterOff();
            break;
        }

        case TCMD_IMU_DEADBAND:
        {
            if (!cmd->hasValue)
            {
                Logger_Log(LOG_INFO, "IMU_DEADBAND,ERR:MISSING_VALUE,OK:0");
                break;
            }
            IMU_GyroFilterSetDeadband(cmd->value);
            break;
        }

        case TCMD_IMU_LPF:
        {
            if (!cmd->hasValue)
            {
                Logger_Log(LOG_INFO, "IMU_LPF,ERR:MISSING_VALUE,OK:0");
                break;
            }
            IMU_GyroFilterSetLpfAlpha(cmd->value);
            break;
        }

        case TCMD_MAGWHO:
        {
            extern I2C_HandleTypeDef hi2c1;
            MAG_QMC5883P_Handle_t mag;
            HAL_StatusTypeDef st = MAG_QMC5883P_Detect(&hi2c1, &mag);
            if (st == HAL_OK && mag.found)
            {
                Logger_Log(LOG_INFO,
                           "MAG_WHO,ADDR:0x%02X,DEVADDR_HAL:0x%02X,"
                           "CHIP:QMC5883P,CHIP_ID:0x%02X,OK:1",
                           (unsigned)mag.addr7,
                           (unsigned)(mag.addr7 << 1),
                           (unsigned)mag.chip_id);
            }
            else if (st == HAL_OK && !mag.found)
            {
                Logger_Log(LOG_INFO, "MAG_WHO,ADDR:0x%02X,OK:0,ERR:NOT_FOUND",
                           (unsigned)MAG_QMC5883P_ADDR7);
            }
            else
            {
                Logger_Log(LOG_INFO, "MAG_WHO,ADDR:0x%02X,OK:0,ERR:NOT_FOUND",
                           (unsigned)MAG_QMC5883P_ADDR7);
            }
            break;
        }

        case TCMD_MAGINIT:
        {
            extern I2C_HandleTypeDef hi2c1;
            static MAG_QMC5883P_Handle_t mag_handle = {0};
            HAL_StatusTypeDef st = MAG_QMC5883P_Init(&hi2c1, &mag_handle);
            if (st == HAL_OK && mag_handle.initialized)
            {
                uint8_t sr = 0, ctrl2 = 0, ctrl1 = 0;
                MAG_QMC5883P_ReadReg(&hi2c1, MAG_QMC5883P_REG_SET_RESET, &sr);
                MAG_QMC5883P_ReadReg(&hi2c1, MAG_QMC5883P_REG_CTRL2, &ctrl2);
                MAG_QMC5883P_ReadReg(&hi2c1, MAG_QMC5883P_REG_CTRL1, &ctrl1);
                Logger_Log(LOG_INFO,
                           "MAG_INIT,CHIP:QMC5883L,ADDR:0x%02X,CHIP_ID:0x%02X,"
                           "SETRST:0x%02X,CTRL2:0x%02X,CTRL1:0x%02X,OK:1",
                           (unsigned)mag_handle.addr7,
                           (unsigned)mag_handle.chip_id,
                           (unsigned)sr, (unsigned)ctrl2, (unsigned)ctrl1);
            }
            else
            {
                const char *err = "UNKNOWN";
                if (!mag_handle.found) err = "NOT_FOUND";
                else if (mag_handle.chip_id != MAG_QMC5883P_CHIP_ID_EXPECTED) err = "CHIP_ID";
                Logger_Log(LOG_INFO, "MAG_INIT,OK:0,ERR:%s", err);
            }
            break;
        }

        case TCMD_MAGRAW:
        {
            extern I2C_HandleTypeDef hi2c1;
            static MAG_QMC5883P_Handle_t mag_handle = {0};
            MAG_QMC5883P_Raw_t raw;
            HAL_StatusTypeDef st = MAG_QMC5883P_ReadRaw(&hi2c1, &mag_handle, &raw);
            if (st == HAL_OK)
            {
                uint8_t drdy = (raw.status & MAG_QMC5883P_STATUS_DRDY) ? 1U : 0U;
                uint8_t ovfl = (raw.status & MAG_QMC5883P_STATUS_OVFL) ? 1U : 0U;
                Logger_Log(LOG_INFO,
                           "MAG_RAW,X:%d,Y:%d,Z:%d,"
                           "STATUS:0x%02X,DRDY:%u,OVFL:%u,CHIP:QMC5883P,OK:1",
                           (int)raw.x, (int)raw.y, (int)raw.z,
                           (unsigned)raw.status, drdy, ovfl);
            }
            else
            {
                Logger_Log(LOG_INFO, "MAG_RAW,OK:0,ERR:READ_FAILED");
            }
            break;
        }

        case TCMD_MAGIMU:
        {
            extern I2C_HandleTypeDef hi2c1;
            static MAG_QMC5883P_Handle_t mag_handle = {0};
            MAG_QMC5883P_ReadImu(&hi2c1, &mag_handle);
            break;
        }

        case TCMD_MAGHELP:
        {
            Logger_Log(LOG_INFO, "MAG_HELP,magwho");
            Logger_Log(LOG_INFO, "MAG_HELP,maginit");
            Logger_Log(LOG_INFO, "MAG_HELP,magraw");
            Logger_Log(LOG_INFO, "MAG_HELP,magimu");
            Logger_Log(LOG_INFO, "MAG_HELP,maghelp");
            break;
        }

        case TCMD_MOTOR_RAW:
        {
            /* Bare motor tag with no payload -> usage error.
             * (DISARM-empty-payload path already returned earlier.) */
            if (cmd->rawPayload[0] == '\0')
            {
                Logger_Log(LOG_ERROR,
                           "Usage: FL <text> | FR <text> | "
                           "RL <text> | RR <text>");
                break;
            }

            Logger_Log(LOG_INFO, "[RAW][%s] %s",
                       MotorTagName(cmd->rawMotor), cmd->rawPayload);

            if (!MotorDispatcher_SendRawToMotor(cmd->rawMotor,
                                                cmd->rawPayload))
            {
                Logger_Log(LOG_ERROR, "Direct motor TX failed for %s",
                           MotorTagName(cmd->rawMotor));
            }
            break;
        }

        case TCMD_MOTOR_TUNE:
        {
            /* Validated tuning command — forward normalised payload. */
            if (!MotorDispatcher_SendTunePayload(cmd->tuneTarget,
                                                 cmd->tunePayload))
            {
                Logger_Log(LOG_ERROR, "[TUNE] Dispatch failed");
            }
            break;
        }

        case TCMD_CFGCACHE:
        {
            if (cmd->cfgMotor == MOTOR_COUNT)
                MotorTuningConfig_PrintAll();
            else
                MotorTuningConfig_Print(cmd->cfgMotor);
            break;
        }

        case TCMD_CFGREAD:
        {
            if (cmd->cfgMotor == MOTOR_COUNT)
            {
                /* cfgread all — send cfg to all four motors */
                MotorDispatcher_SendRawToMotor(MOTOR_FL, "cfg");
                MotorDispatcher_SendRawToMotor(MOTOR_FR, "cfg");
                MotorDispatcher_SendRawToMotor(MOTOR_RL, "cfg");
                MotorDispatcher_SendRawToMotor(MOTOR_RR, "cfg");
                Logger_Log(LOG_INFO, "[CFGREAD] sent cfg to all motors");
            }
            else
            {
                /* cfgread single motor — e.g. "RL cfg" */
                const char *tag = "??";
                switch (cmd->cfgMotor)
                {
                    case MOTOR_FL: tag = "FL"; break;
                    case MOTOR_FR: tag = "FR"; break;
                    case MOTOR_RL: tag = "RL"; break;
                    case MOTOR_RR: tag = "RR"; break;
                    default: break;
                }
                MotorDispatcher_SendRawToMotor(cmd->cfgMotor, "cfg");
                Logger_Log(LOG_INFO, "[CFGREAD] sent cfg to %s", tag);
            }
            break;
        }

        case TCMD_ARM_RAW:
        {
            if (cmd->armPayload[0] == '\0')
            {
                Logger_Log(LOG_ERROR, "Usage: arm <payload>");
                break;
            }
            if (ManipulationUartDma_SendRaw(cmd->armPayload))
                Logger_Log(LOG_INFO, "[ARM] queued");
            else
                Logger_Log(LOG_ERROR, "[ARM] UART8 TX queue failed");
            break;
        }

        default:
            break;
    }
}
