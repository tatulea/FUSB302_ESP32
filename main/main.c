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
#include "defines.h"

static DevicePolicyPtr_t dpm;
static Port_t ports[1]; 

volatile bool fusb_ready = false;
esp_timer_handle_t fusb_timer;


static void IRAM_ATTR fusb_isr_handler(void* arg)
{
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
        .sda_io_num=I2C_SDA,
        .scl_io_num=I2C_SCL,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    i2c_set_timeout(I2C_NUM_0, 400000);

    gpio_set_direction(FUSB_INT, GPIO_MODE_INPUT);
    gpio_set_intr_type(FUSB_INT, GPIO_INTR_NEGEDGE);
    gpio_set_pull_mode(FUSB_INT, GPIO_PULLUP_ONLY);

    // Install gpio ISR
    gpio_install_isr_service(0);
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(FUSB_INT, fusb_isr_handler, (void*) FUSB_INT);

    DPM_Init(&dpm);

    ports[0].dpm = dpm;
    ports[0].PortID = 0;
    core_initialize(&ports[0], FUSB_ADDR);

    DPM_AddPort(dpm, &ports[0]);

    const esp_timer_create_args_t fusb_timer_args = {
        .callback = &oneshot_timer_callback,
        .name = "fusb-timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&fusb_timer_args, &fusb_timer));

    DELAY(2000);
    
    while(1)
    {
        if (fusb_ready)
        {
            printf("Got an interrupt\r\n");
            esp_timer_stop(fusb_timer);
            core_state_machine(&ports[0]);

            fusb_ready = false;

            /*
             * It is possible for the state machine to go into idle mode with
             * the interrupt pin still low and as a result the edge-sensitive
             * IRQ won't get triggered.  Check here before returning to wait
             * on the IRQ.
             */
            if(platform_get_device_irq_state(ports[0].PortID))
            {
                fusb_ready = true;
            }
            else
            {
                /* If needed, enable timer interrupt before idling */
                uint32_t timer_value = core_get_next_timeout(&ports[0]);

                if (timer_value > 0)
                {
                    printf("Timer is bigger than 0\r\n");
                    if (timer_value == 1)
                    {
                        /* A value of 1 indicates that a timer has expired
                         * or is about to expire and needs further processing.
                         */
                        printf("Timer got an interrupt\r\n");
                        fusb_ready = true;
                    }
                    else
                    {
                        /* A non-zero time requires a future timer interrupt */
                        printf("Timer reset\r\n");
                        ESP_ERROR_CHECK(esp_timer_start_once(fusb_timer, timer_value));
                    }
                }
                else
                {
                    /* Optional: Disable system timer(s) here to save power
                     * while in Idle mode.
                     *
                     * Note: this is the place that gets called when the FUSB
                     * finished the process for an interrupt. Here you will know
                     * for sure that you have the final state of USB
                     */
                    printf("FUSB process done. You can query the state here\r\n");
                }
            }
        }
    }
}
