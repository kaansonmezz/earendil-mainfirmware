#include "motor_link.h"
#include "app_config.h"
#include <string.h>

/* ── Public functions ───────────────────────────────────────────────────── */

void MotorLink_Init(MotorLink_t *link, MotorId_t id)
{
    memset(link, 0, sizeof(MotorLink_t));
    link->id    = id;
    link->state = LINK_IDLE;
}

void MotorLink_Update(MotorLink_t *link)
{
    if (link->state == LINK_WAIT_ACK)
    {
        uint32_t elapsed = HAL_GetTick() - link->lastTxTick;
        if (elapsed >= ACK_TIMEOUT_MS)
        {
            link->state = LINK_TIMEOUT;
        }
    }
}
