#include "motor_dispatcher.h"
#include "motor_protocol.h"
#include "control_mode.h"
#include "app_config.h"
#include "motor_tx_dma.h"
#include "operating_mode.h"
#include "logger.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

static MotorLink_t motorLinks[MOTOR_COUNT];
static char        txBuf[32];

void MotorDispatcher_Init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        MotorLink_Init(&motorLinks[i], (MotorId_t)i);
    }
}

void MotorDispatcher_Send(MotorId_t id, const MotorCmd_t *cmd)
{
    if (id >= MOTOR_COUNT || cmd == NULL)
        return;

    /* Defense-in-depth: while DISARM is active, only STOP/zero framed motor
     * commands may reach the UARTs.  Any FORWARD/BACKWARD frame is dropped
     * here even if a caller bypassed the command_handler gate. */
    if (OperatingMode_IsDisarm() && cmd->dir != MCMD_STOP)
    {
        Logger_Log(LOG_WARN, "[DISARM] motor dispatch blocked (motor %d)", (int)id);
        return;
    }

    uint16_t frameLen = MotorProtocol_Encode(cmd, txBuf, sizeof(txBuf));

    if (frameLen == 0)
        return;

    motorLinks[id].lastTxTick = HAL_GetTick();
    motorLinks[id].state      = LINK_WAIT_ACK;
    motorLinks[id].retryCount = 0;

    if (!MotorTxDma_Send(id, txBuf))
    {
        Logger_Log(LOG_ERROR, "UART TX failed for motor %d", id);
        return;
    }

    const char *names[] = {"FL", "FR", "RL", "RR"};
    bool isRpm = (ControlMode_Get() == CONTROL_MODE_RPM);

    if (cmd->dir == MCMD_STOP)
    {
        Logger_Log(LOG_INFO, "[TX][%s] stop", names[id]);
    }
    else if (cmd->dir == MCMD_FORWARD)
    {
        if (isRpm)
            Logger_Log(LOG_INFO, "[TX][%s] rpm %u", names[id], cmd->pwm);
        else
            Logger_Log(LOG_INFO, "[TX][%s] f%u", names[id], cmd->pwm);
    }
    else if (cmd->dir == MCMD_BACKWARD)
    {
        if (isRpm)
            Logger_Log(LOG_INFO, "[TX][%s] rpm -%u", names[id], cmd->pwm);
        else
            Logger_Log(LOG_INFO, "[TX][%s] b%u", names[id], cmd->pwm);
    }
}

void MotorDispatcher_SendAll(const MotorCmd_t cmds[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        MotorDispatcher_Send((MotorId_t)i, &cmds[i]);
    }
}

void MotorDispatcher_SendRaw(const char *msg)
{
    if (msg == NULL)
        return;

    const char *names[] = {"FL", "FR", "RL", "RR"};

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        char frame[64];
        int len = snprintf(frame, sizeof(frame), "%s\r\n", msg);
        if (len <= 0 || (uint16_t)len >= sizeof(frame))
            continue;

        if (MotorTxDma_Send((MotorId_t)i, frame))
        {
            Logger_Log(LOG_INFO, "[TX][%s] %s", names[i], msg);
        }
        else
        {
            Logger_Log(LOG_ERROR, "UART TX failed for motor %s raw", names[i]);
        }
    }
}

MotorLink_t *MotorDispatcher_GetLink(MotorId_t id)
{
    if (id >= MOTOR_COUNT)
        return NULL;

    return &motorLinks[id];
}

void MotorDispatcher_Update(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        MotorLink_Update(&motorLinks[i]);
    }
}
