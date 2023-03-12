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
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include "dht22.h"


extern "C" void app_main(void)
{
    dht22::Dht22Sensor sensor4 = dht22::Dht22Sensor((gpio_num_t)4);    
    dht22::Dht22Sensor sensor27 = dht22::Dht22Sensor((gpio_num_t)27);
    struct dht22::SensorReadingResult result;

    while(true)
    {
        sensor4.read(&result);
        printf("Sensor4 => Humidity:%.1f Temperature:%.1f ResultCode:%d\n",result.Humidity,result.Temperature,result.ResultCode);

        sensor27.read(&result);
        printf("Sensor27 => Humidity:%.1f Temperature:%.1f ResultCode:%d\n",result.Humidity,result.Temperature,result.ResultCode);

	    vTaskDelay(5000 / portTICK_PERIOD_MS);        
    }

}
