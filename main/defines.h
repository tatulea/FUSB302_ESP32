#pragma once

#include "freertos/FreeRTOS.h"


#define DELAY(__attr__)         vTaskDelay(__attr__ / portTICK_PERIOD_MS);

#define I2C_SCL     14
#define I2C_SDA     27
#define FUSB_INT    12
#define FUSB_ADDR   0x25