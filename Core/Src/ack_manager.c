#include "ack_manager.h"
#include "motor_dispatcher.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>

/* ── Private types ──────────────────────────────────────────────────────── */
typedef struct
{
    bool     pending;
    uint32_t sentTick;
} PendingAck_t;

/* ── Private variables ──────────────────────────────────────────────────── */
static PendingAck_t pendingAcks[MOTOR_COUNT];

/* ── Public functions ───────────────────────────────────────────────────── */

void AckManager_Init(void)
{
    memset(pendingAcks, 0, sizeof(pendingAcks));
}

void AckManager_RegisterPending(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return;

    pendingAcks[id].pending  = true;
    pendingAcks[id].sentTick = HAL_GetTick();
}

AckStatus_t AckManager_CheckStatus(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return ACK_ERROR;

    MotorLink_t *link = MotorDispatcher_GetLink(id);
    if (link == NULL)
        return ACK_ERROR;

    if (link->state == LINK_ACKED)
    {
        link->state = LINK_IDLE;
        pendingAcks[id].pending = false;
        return ACK_OK;
    }

    if (link->state == LINK_TIMEOUT)
    {
        pendingAcks[id].pending = false;
        link->state = LINK_IDLE;
        return ACK_TIMEOUT;
    }

    return ACK_NONE;
}

void AckManager_Update(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
    	if (!pendingAcks[i].pending)
            continue;

        uint32_t elapsed = HAL_GetTick() - pendingAcks[i].sentTick;

        if (elapsed >= ACK_TIMEOUT_MS)
        {
            MotorLink_t *link = MotorDispatcher_GetLink((MotorId_t)i);
            if (link != NULL && link->retryCount < MAX_RETRIES)
            {
                link->retryCount++;
                link->lastTxTick = HAL_GetTick();
                link->state      = LINK_WAIT_ACK;
                pendingAcks[i].sentTick = HAL_GetTick();

                Logger_Log(LOG_WARN, "ACK timeout motor %d, retry %d/%d",
                           i, link->retryCount, MAX_RETRIES);
            }
            else
            {
                pendingAcks[i].pending = false;
                Logger_Log(LOG_ERROR, "ACK FAIL motor %d after %d retries",
                           i, MAX_RETRIES);
            }
        }
    }
}
