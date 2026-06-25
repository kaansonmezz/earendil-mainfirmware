/*
 * activity_light.c
 *
 *  Created on: Jun 18, 2026
 *      Author: Emirhan
 */
#include "activity_light.h"

// TODO: CubeMX üzerinden pinleri atadıktan sonra buradaki makroları güncelleyebilirsin.
// Şimdilik örnek olarak B portundaki pinleri tanımlıyorum.
#define LIGHT_RED_PORT    GPIOB
#define LIGHT_RED_PIN     GPIO_PIN_0
#define LIGHT_GREEN_PORT  GPIOB
#define LIGHT_GREEN_PIN   GPIO_PIN_1
#define LIGHT_YELLOW_PORT GPIOB
#define LIGHT_YELLOW_PIN  GPIO_PIN_2

// Mevcut rover modunu takip eden durum değişkeni
static RoverMode_t s_roverMode = ROVER_MODE_DISARM;

void ActivityLight_Init(void) {
    // CubeMX donanım başlatma işlemlerini (MX_GPIO_Init) main içerisinde zaten yapıyor.
    // Biz sadece sistemi ilk açtığımızda güvenlik gereği doğrudan "DISARM" (Kırmızı) modunda başlamasını sağlıyoruz.
    ActivityLight_SetMode(ROVER_MODE_DISARM);
}

void ActivityLight_SetMode(RoverMode_t mode) {
    // Önce tüm ışıkları söndür (Eğer röleler Low-Trigger ise RESET ve SET mantığını tersine çevirmelisin)
    HAL_GPIO_WritePin(LIGHT_RED_PORT, LIGHT_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIGHT_GREEN_PORT, LIGHT_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIGHT_YELLOW_PORT, LIGHT_YELLOW_PIN, GPIO_PIN_RESET);

    // İstenen moda göre sadece ilgili ışığı yak
    switch (mode) {
        case ROVER_MODE_DISARM:
            HAL_GPIO_WritePin(LIGHT_RED_PORT, LIGHT_RED_PIN, GPIO_PIN_SET);
            break;
        case ROVER_MODE_MANUAL:
            HAL_GPIO_WritePin(LIGHT_GREEN_PORT, LIGHT_GREEN_PIN, GPIO_PIN_SET);
            break;
        case ROVER_MODE_AUTONOMOUS:
            HAL_GPIO_WritePin(LIGHT_YELLOW_PORT, LIGHT_YELLOW_PIN, GPIO_PIN_SET);
            break;
        default:
            // Beklenmeyen veya hatalı bir mod gelirse sistemi güvenlik (Kırmızı) moduna çek
            HAL_GPIO_WritePin(LIGHT_RED_PORT, LIGHT_RED_PIN, GPIO_PIN_SET);
            mode = ROVER_MODE_DISARM;
            break;
    }

    s_roverMode = mode;
}

RoverMode_t ActivityLight_GetMode(void) {
    return s_roverMode;
}

const char *ActivityLight_ToString(RoverMode_t mode) {
    switch (mode) {
        case ROVER_MODE_DISARM:     return "disarm";
        case ROVER_MODE_MANUAL:     return "manual";
        case ROVER_MODE_AUTONOMOUS: return "autonomous";
        default:                    return "disarm";
    }
}

