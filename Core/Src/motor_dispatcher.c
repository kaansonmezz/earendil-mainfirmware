#include "motor_dispatcher.h"
#include "motor_protocol.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>
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

    uint16_t frameLen = MotorProtocol_Encode(cmd, txBuf, sizeof(txBuf));

    if (frameLen == 0)
        return;

    UART_HandleTypeDef *huart = MOTOR_UART_HANDLE(id);

    if (huart == NULL)
        return;

    motorLinks[id].lastTxTick = HAL_GetTick();
    motorLinks[id].state      = LINK_WAIT_ACK;
    motorLinks[id].retryCount = 0;

    if (HAL_UART_Transmit(huart, (uint8_t *)txBuf, frameLen, 100) != HAL_OK)
    {
        Logger_Log(LOG_ERROR, "UART TX failed for motor %d", id);
        return;
    }

    const char *names[] = {"FL", "FR", "RL", "RR"};

    if (cmd->dir == MCMD_STOP)
    {
        Logger_Log(LOG_INFO, "[TX][%s] stop", names[id]);
    }
    else if (cmd->dir == MCMD_FORWARD)
    {
        Logger_Log(LOG_INFO, "[TX][%s] f%u", names[id], cmd->pwm);
    }
    else if (cmd->dir == MCMD_BACKWARD)
    {
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
        UART_HandleTypeDef *huart = MOTOR_UART_HANDLE((MotorId_t)i);
        if (huart == NULL)
            continue;

        char frame[64];
        int len = snprintf(frame, sizeof(frame), "%s\r\n", msg);
        if (len <= 0 || (uint16_t)len >= sizeof(frame))
            continue;

        HAL_UART_Transmit(huart, (uint8_t *)frame, (uint16_t)len, 100);
        Logger_Log(LOG_INFO, "[TX][%s] %s", names[i], msg);
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
