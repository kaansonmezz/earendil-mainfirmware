#include "motor_protocol.h"
#include <stdio.h>
#include <string.h>

uint16_t MotorProtocol_Encode(const MotorCmd_t *cmd, char *buf, uint16_t bufSize)
{
    if (cmd == NULL || buf == NULL || bufSize < 8)
        return 0;

    int len = 0;

    switch (cmd->dir)
    {
        case MCMD_FORWARD:
            len = snprintf(buf, bufSize, "f%u\r\n", cmd->pwm);
            break;

        case MCMD_BACKWARD:
            len = snprintf(buf, bufSize, "b%u\r\n", cmd->pwm);
            break;

        case MCMD_STOP:
            len = snprintf(buf, bufSize, "stop\r\n");
            break;

        default:
            return 0;
    }

    if (len <= 0 || len >= (int)bufSize)
        return 0;

    return (uint16_t)len;
}
