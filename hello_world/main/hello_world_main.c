/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"

#define  GREEN_GPIO 17
#define  RED_GPIO 16
//static uint8_t s_led_state = 0;

#define STACK_SIZE 2048

struct LedInfo
{
    int pinNo;
    TaskHandle_t parent;
};

void blinkTask( void * pvParameters )
{
    struct LedInfo *ledInfo = pvParameters;
    int pinNo = ledInfo->pinNo;
    TaskHandle_t parent = ledInfo->parent;
    uint32_t signalV;

    uint8_t ledState = 0;

    gpio_reset_pin(pinNo);
    gpio_set_direction(pinNo, GPIO_MODE_OUTPUT);

    while(true)
    {
//        printf("Waiting for signal\n");    

        xTaskNotifyWait(0x00,ULONG_MAX,&signalV,portMAX_DELAY);

//        printf("Got for signal\n");
        // Task code goes here.
        gpio_set_level(pinNo, ledState);      
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ledState = !ledState;
    }
}

void app_main(void)
{
    static struct LedInfo lInfo;

    lInfo.pinNo = GREEN_GPIO;
    lInfo.parent = xTaskGetCurrentTaskHandle();

    TaskHandle_t xHandle = NULL;
    uint32_t signalV;

  // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
  // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
  // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
  // the new task attempts to access it.
  xTaskCreate( &blinkTask, "Blink1", STACK_SIZE, &lInfo, tskIDLE_PRIORITY, &xHandle );
  configASSERT( xHandle );

    while(true)
    {
        printf("Sending signal\n");    

        xTaskNotify(xHandle,10,eNoAction);
        printf("Sleeping\n");    
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
