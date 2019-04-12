#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"

#include "driver/i2c.h"
#include "driver/timer.h"

#include "core.h"
#include "Port.h"

#define DELAY(__attr__)         vTaskDelay(__attr__ / portTICK_PERIOD_MS);


static DevicePolicyPtr_t dpm;
static Port_t ports[1]; 


volatile bool fusb_ready = false;
#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_INTERVAL1_SEC   1

volatile bool timer_intr = false;

void IRAM_ATTR timer_group0_isr(void *para)
{
    TIMERG0.int_clr_timers.t1 = 1;
    TIMERG0.hw_timer[TIMER_1].config.alarm_en = TIMER_ALARM_EN;
    timer_intr = true;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if((uint32_t) arg == 35)
        fusb_ready = true;
}

void init_timer()
{
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    timer_init(TIMER_GROUP_0, TIMER_1, &config);

    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL);

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, TIMER_INTERVAL1_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_group0_isr, 
        (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, TIMER_1);
}

void app_main()
{
    init_timer();

    i2c_config_t i2c_config = {
        .mode=I2C_MODE_MASTER,
        .sda_io_num=25,
        .scl_io_num=26,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=1000000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    gpio_set_direction(35, GPIO_MODE_INPUT);
    gpio_set_intr_type(35, GPIO_INTR_NEGEDGE);

    // Install gpio ISR
    gpio_install_isr_service(0);
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(35, gpio_isr_handler, (void*) 35);


    DPM_Init(&dpm);

    ports[0].dpm = dpm;
    ports[0].PortID = 0;
    core_initialize(&ports[0], 0x22);

    DPM_AddPort(dpm, &ports[0]);

    while(1)
    {
        if (fusb_ready == true)
        {
            timer_pause(TIMER_GROUP_0, TIMER_1);
            core_state_machine(&ports[0]);

            fusb_ready = false;
            
            if(gpio_get_level(35) == 0)
            {
                fusb_ready = true;
            }
            else
            {
                double elapsed;
                timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_1, &elapsed);

                if (elapsed > 0)
                {
                    if (elapsed > 0.9f)
                    {
                        /* A value of 1 indicates that a timer has expired
                         * or is about to expire and needs further processing.
                         */
                        fusb_ready = true;
                    }
                    else
                    {
                        timer_start(TIMER_GROUP_0, TIMER_1);
                    }
                }
                else
                {
                    /* Optional: Disable system timer(s) here to save power
                     * if needed while in Idle mode.
                     */
                }
            }

            printf("CC state: %d\r\n", core_get_cc_orientation(&ports[0]));
        }
    }
}
