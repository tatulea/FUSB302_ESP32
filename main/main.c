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

#include "esp_timer.h"

#define DELAY(__attr__)         vTaskDelay(__attr__ / portTICK_PERIOD_MS);
#define INT_PIN     12

static DevicePolicyPtr_t dpm;
static Port_t ports[1]; 


volatile bool fusb_ready = false;
#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_INTERVAL1_SEC   1

volatile bool timer_intr = false;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if((uint32_t) arg == INT_PIN)
        fusb_ready = true;
}

static void oneshot_timer_callback(void* arg)
{
    fusb_ready = true;
}

void app_main()
{
    i2c_config_t i2c_config = {
        .mode=I2C_MODE_MASTER,
        .sda_io_num=27,
        .scl_io_num=14,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=1000000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    gpio_set_direction(INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(INT_PIN, GPIO_INTR_NEGEDGE);
    gpio_set_pull_mode(INT_PIN, GPIO_PULLUP_ONLY);

    // Install gpio ISR
    gpio_install_isr_service(0);
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(INT_PIN, gpio_isr_handler, (void*) INT_PIN);

    DPM_Init(&dpm);

    ports[0].dpm = dpm;
    ports[0].PortID = 0;
    core_initialize(&ports[0], 0x22);

    DPM_AddPort(dpm, &ports[0]);

    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &oneshot_timer_callback,
        .name = "one-shot"
    };
    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

    while(1)
    {
        if (fusb_ready)
        {
            esp_timer_stop(oneshot_timer);
            // esp_timer_delete(oneshot_timer);
            core_state_machine(&ports[0]);

            fusb_ready = false;

            if(gpio_get_level(INT_PIN) == 0)
            {
                fusb_ready = true;
            }
            else
            {
                /* If needed, enable timer interrupt before idling */
                uint32_t timer_value = core_get_next_timeout(&ports[0]);
                printf("Timer value: %d\r\n", timer_value);
                if (timer_value > 0)
                {
                    if (timer_value == 1)
                    {
                        /* A value of 1 indicates that a timer has expired
                         * or is about to expire and needs further processing.
                         */
                        fusb_ready = true;
                    }
                    else
                    {
                        // ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
                        ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, timer_value));
                    }
                }
                else
                {
                    /* Optional: Disable system timer(s) here to save power
                     * if needed while in Idle mode.
                     */
                }
            }

            printf("CC state: %d\r\n", ports[0].CCPin);
            switch (ports[0].ConnState)
            {
                case Unattached:
                {
                    printf("The cable is unattached\r\n");
                    break;
                }
                case AttachedSink:
                {
                    printf("Source in sink state\r\n");
                    break;
                }
                case AttachedSource:
                {
                    printf("Host in attached source state\r\n");
                    break;
                }
                case AttachWaitSource:
                {
                    printf("Host in attach wait source state\r\n");
                    break;
                }
                default:
                {
                    printf("The state is not recognized. Value %d\r\n", ports[0].ConnState);
                    printf("CC state: %d\r\n", ports[0].CCPin);
                    printf("CC TermType: %d\r\n", ports[0].CCTerm);
                    printf("SRC/SNK: %d\r\n", ports[0].sourceOrSink);
                    break;
                }
            }
        }
    }
}
