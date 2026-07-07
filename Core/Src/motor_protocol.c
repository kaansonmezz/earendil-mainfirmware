#include "motor_protocol.h"
#include "control_mode.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

uint16_t MotorProtocol_Encode(const MotorCmd_t *cmd, char *buf, uint16_t bufSize)
{
    if (cmd == NULL || buf == NULL || bufSize < 12)
        return 0;

    int len = 0;
    bool isRpm = (ControlMode_Get() == CONTROL_MODE_RPM);

    switch (cmd->dir)
    {
        case MCMD_FORWARD:
            if (isRpm)
                len = snprintf(buf, bufSize, "rpm %u\r\n", cmd->pwm);
            else
                len = snprintf(buf, bufSize, "f%u\r\n", cmd->pwm);
            break;

        case MCMD_BACKWARD:
            if (isRpm)
                len = snprintf(buf, bufSize, "rpm -%u\r\n", cmd->pwm);
            else
                len = snprintf(buf, bufSize, "b%u\r\n", cmd->pwm);
            break;

        case MCMD_STOP:
            len = snprintf(buf, bufSize, "stop\r\n");
            break;

        case MCMD_BRAKE:
            len = snprintf(buf, bufSize, "x\r\n");
            break;

        default:
            return 0;
    }

    if (len <= 0 || len >= (int)bufSize)
        return 0;

    return (uint16_t)len;
}
