#include "safety_manager.h"
#include "motion_controller.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>

/* ── Private variables ──────────────────────────────────────────────────── */
static uint32_t lastRxTick[MOTOR_COUNT];
static bool     linkLost[MOTOR_COUNT];

/* ── Public functions ───────────────────────────────────────────────────── */

void SafetyManager_Init(void)
{
    memset(lastRxTick, 0, sizeof(lastRxTick));
    memset(linkLost, false, sizeof(linkLost));
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
                Logger_Log(LOG_ERROR, "LINK LOST motor %d", i);
                MotionController_Stop();
            }
        }
    }
}

void SafetyManager_NotifyRx(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return;

    lastRxTick[id] = HAL_GetTick();
    linkLost[id]   = false;
}

bool SafetyManager_IsLinkLost(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return true;
    return linkLost[id];
}
