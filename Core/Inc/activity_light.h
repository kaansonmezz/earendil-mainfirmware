/*
 * activity_light.h
 *
 *  Created on: Jun 18, 2026
 *      Author: Emirhan
 */

#ifndef INC_ACTIVITY_LIGHT_H_
#define INC_ACTIVITY_LIGHT_H_

#include "stm32h7xx_hal.h" // STM32H7 HAL Kütüphanesi

// Rover çalışma modlarını tanımlayan Enum
typedef enum {
    ROVER_MODE_DISARM = 0, // Kırmızı Işık
    ROVER_MODE_MANUAL,     // Yeşil Işık
    ROVER_MODE_AUTONOMOUS  // Sarı Işık
} RoverMode_t;

// Dışarıdan çağırılacak fonksiyonlar
void ActivityLight_Init(void);
void ActivityLight_SetMode(RoverMode_t mode);
RoverMode_t ActivityLight_GetMode(void);
const char *ActivityLight_ToString(RoverMode_t mode);


#endif /* INC_ACTIVITY_LIGHT_H_ */
