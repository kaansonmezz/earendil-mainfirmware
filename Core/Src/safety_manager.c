#include "safety_manager.h"
#include "app_config.h"
#include "logger.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "motor_tx_dma.h"
#include "operating_mode.h"

/* ── Private variables ──────────────────────────────────────────────────── */
/* volatile: written from SafetyManager_NotifyRx() (RX callback/ISR context),
 * read from SafetyManager_Update() (main loop). */
static volatile uint32_t lastRxTick[MOTOR_COUNT];
static volatile bool     linkLost[MOTOR_COUNT];
static volatile bool     recoveryPending[MOTOR_COUNT];

/* ── PC/Pi control-link watchdog state ────────────────────────────────────
 *  Updated from main-loop context only (command_handler calls
 *  SafetyManager_NotifyPcActivity() or SafetyManager_NotifyPcHeartbeat()).
 *  The periodic check runs in SafetyManager_Update() which is also
 *  main-loop context, so no volatile qualifier is required. */
static uint32_t pcLastActivityMs;
static bool     pcLinkSeen;
static bool     pcLinkAlive;
static bool     pcLinkTimeoutLatched;
static bool     pcRecoveryPending;

/* ── PC heartbeat freshness (separate from activity freshness) ────────────
 *  Only TCMD_HEARTBEAT updates this state.  Motion, stop, brake, and
 *  mode commands refresh pcLastActivityMs but NOT pcLastHeartbeatMs.
 *  MANUAL mode entry requires a fresh heartbeat — not merely fresh
 *  activity — so that stale motion commands cannot authorize MANUAL. */
static bool     pcHeartbeatSeen;
static uint32_t pcLastHeartbeatMs;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static const char *GetMotorName(MotorId_t id)
{
    switch (id)
    {
        case MOTOR_FL: return "FL";
        case MOTOR_FR: return "FR";
        case MOTOR_RL: return "RL";
        case MOTOR_RR: return "RR";
        default:       return "??";
    }
}

/* ── Public functions ───────────────────────────────────────────────────── */

void SafetyManager_Init(void)
{
    for (MotorId_t i = 0; i < MOTOR_COUNT; i++)
    {
        lastRxTick[i]     = 0U;
        linkLost[i]       = false;
        recoveryPending[i] = false;
    }

    /* PC/Pi control-link watchdog */
    pcLastActivityMs     = 0U;
    pcLinkSeen           = false;
    pcLinkAlive          = false;
    pcLinkTimeoutLatched = false;
    pcRecoveryPending    = false;

    /* PC heartbeat freshness */
    pcHeartbeatSeen      = false;
    pcLastHeartbeatMs    = 0U;
}

void SafetyManager_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ── Check link loss for each motor ─────────────────────────────── */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (lastRxTick[i] == 0)
            continue; /* no RX yet, skip */

        if ((now - lastRxTick[i]) >= LINK_LOSS_TIMEOUT_MS)
        {
            if (!linkLost[i])
            {
                linkLost[i] = true;
                Logger_Log(LOG_ERROR, "[LINK_LOST][%s] No RX for %lu ms",
                           GetMotorName((MotorId_t)i),
                           (unsigned long)LINK_LOSS_TIMEOUT_MS);
            }
        }
    }

    /* ── Deferred recovery log (set from ISR, logged here in main loop) */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (recoveryPending[i])
        {
            recoveryPending[i] = false;
            Logger_Log(LOG_INFO, "[LINK_RECOVERED][%s] link reestablished",
                       GetMotorName((MotorId_t)i));
        }
    }

    /* ── PC/Pi control-link watchdog ──────────────────────────────────────
     *  Only fires once per timeout transition (latch prevents repeat).
     *  Three-branch behaviour:
     *    MANUAL     → STOP + DISARM (safety-critical: rover was moving)
     *    AUTONOMOUS → mark link stale, no motor effect
     *    DISARM     → no-op, motor safety sequence not re-run */
    if (pcLinkSeen && pcLinkAlive && !pcLinkTimeoutLatched)
    {
        if ((uint32_t)(now - pcLastActivityMs) >= PC_LINK_TIMEOUT_MS)
        {
            pcLinkTimeoutLatched = true;
            pcLinkAlive = false;

            switch (OperatingMode_Get())
            {
                case ROVER_MODE_MANUAL:
                    SafetyManager_EnterDisarm();
                    OperatingMode_Set(ROVER_MODE_DISARM);
                    Logger_Log(LOG_ERROR,
                               "[PC_LINK] TIMEOUT,AGE_MS:%lu,ACTION:STOP_DISARM",
                               (unsigned long)(now - pcLastActivityMs));
                    break;

                case ROVER_MODE_AUTONOMOUS:
                    Logger_Log(LOG_WARN,
                               "[PC_LINK] TIMEOUT,AGE_MS:%lu,ACTION:LINK_STALE",
                               (unsigned long)(now - pcLastActivityMs));
                    break;

                case ROVER_MODE_DISARM:
                default:
                    Logger_Log(LOG_INFO,
                               "[PC_LINK] TIMEOUT,AGE_MS:%lu,ACTION:NONE",
                               (unsigned long)(now - pcLastActivityMs));
                    break;
            }
        }
    }

    /* ── Deferred PC-link recovery log (main-loop context) ────────────── */
    if (pcRecoveryPending)
    {
        pcRecoveryPending = false;
        Logger_Log(LOG_INFO, "[PC_LINK] RECOVERED");
    }
}

void SafetyManager_NotifyRx(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return;

    /* If this motor was marked as link-lost, flag recovery for main-loop log.
     * Cannot log here — called from RX callback/ISR context. */
    if (linkLost[id])
        recoveryPending[id] = true;

    lastRxTick[id] = HAL_GetTick();
    linkLost[id]   = false;
}

bool SafetyManager_IsLinkLost(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return true;
    return linkLost[id];
}

/* ── DISARM safety lock helpers ──────────────────────────────────────────── */

void SafetyManager_EnterDisarm(void)
{
    /* Hard brake sequence on DISARM: an abrupt stop to every motor.
     *
     *   1. Drop any motion frame that was staged before DISARM (so it
     *      cannot sneak in between the brake and the stop).
     *   2. Send a hard brake ("x") to all motor UARTs — abrupt stop.
     *   3. Send a framed STOP/0 to all motor UARTs — hold at zero.  If a
     *      channel is still busy transmitting "x", this STOP is staged as
     *      pending and fires automatically on TX-complete (safety commands
     *      always win the pending slot), so the order x-then-stop is preserved.
     *   4. Zero the internal motion command table so nothing stale remains.
     *
     * The defense-in-depth gate in motor_dispatcher blocks any non-STOP
     * frame while disarmed, so only these safety frames reach the UARTs. */
    MotorTxDma_CancelPending();       /* drop staged stale motion first */
    MotorDispatcher_SendRaw("x");     /* hard brake: abrupt stop to all motors */
    MotionController_Stop();          /* framed STOP/0 to all motor UARTs */
    MotionController_DisarmSafe();    /* zero the internal motion command table */
}

void SafetyManager_LeaveDisarm(void)
{
    /* Leaving DISARM must NOT inherit stale motion.  Keep motors stopped and
     * clear any leftover state; a fresh valid motion command is required to
     * move.  Nothing is sent on the UARTs here — motors are already stopped. */
    MotorTxDma_CancelPending();
    MotionController_DisarmSafe();
}

/* ── PC/Pi control-link watchdog public API ──────────────────────────────── */

void SafetyManager_NotifyPcActivity(void)
{
    uint32_t now = HAL_GetTick();

    /* If recovering from a timeout, flag the recovery log */
    if (pcLinkTimeoutLatched || (pcLinkSeen && !pcLinkAlive))
        pcRecoveryPending = true;

    pcLastActivityMs     = now;
    pcLinkSeen           = true;
    pcLinkAlive          = true;
    pcLinkTimeoutLatched = false;
}

bool SafetyManager_IsPcLinkAlive(void)
{
    return pcLinkAlive;
}

bool SafetyManager_IsPcLinkSeen(void)
{
    return pcLinkSeen;
}

bool SafetyManager_IsPcLinkTimeout(void)
{
    return pcLinkTimeoutLatched;
}

uint32_t SafetyManager_PcLinkAgeMs(void)
{
    if (pcLastActivityMs == 0U)
        return 0xFFFFFFFFU;  /* never received */
    return (uint32_t)(HAL_GetTick() - pcLastActivityMs);
}

/* ── PC heartbeat freshness (separate from activity freshness) ───────────── */

void SafetyManager_NotifyPcHeartbeat(void)
{
    uint32_t now = HAL_GetTick();

    /* If recovering from a timeout, flag the recovery log */
    if (pcLinkTimeoutLatched || (pcLinkSeen && !pcLinkAlive))
        pcRecoveryPending = true;

    pcLastActivityMs     = now;  /* also keeps the activity watchdog alive */
    pcLastHeartbeatMs    = now;
    pcLinkSeen           = true;
    pcLinkAlive          = true;
    pcLinkTimeoutLatched = false;
    pcHeartbeatSeen      = true;
}

bool SafetyManager_IsPcHeartbeatFresh(void)
{
    if (!pcHeartbeatSeen)
        return false;
    return ((uint32_t)(HAL_GetTick() - pcLastHeartbeatMs)) < PC_LINK_TIMEOUT_MS;
}
