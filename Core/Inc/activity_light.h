/*
 * activity_light.h
 *
 *  Created on: Jun 18, 2026
 *      Author: Emirhan
 */

#ifndef INC_ACTIVITY_LIGHT_H_
#define INC_ACTIVITY_LIGHT_H_

#include "stm32h7xx_hal.h" /* STM32H7 HAL Library */
#include "rover_types.h"   /* RoverMode_t (canonical home) */

/* Rover operating modes are declared in rover_types.h (RoverMode_t). */

// Dışarıdan çağırılacak fonksiyonlar
void ActivityLight_Init(void);
void ActivityLight_SetMode(RoverMode_t mode);
RoverMode_t ActivityLight_GetMode(void);
const char *ActivityLight_ToString(RoverMode_t mode);


#endif /* INC_ACTIVITY_LIGHT_H_ */
