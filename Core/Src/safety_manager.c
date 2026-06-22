#include "safety_manager.h"
#include "app_config.h"
#include "logger.h"

/* ── Private variables ──────────────────────────────────────────────────── */
/* volatile: written from SafetyManager_NotifyRx() (RX callback/ISR context),
 * read from SafetyManager_Update() (main loop). */
static volatile uint32_t lastRxTick[MOTOR_COUNT];
static volatile bool     linkLost[MOTOR_COUNT];
static volatile bool     recoveryPending[MOTOR_COUNT];

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
