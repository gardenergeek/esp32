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

/*
#define  DHT22PIN 4
#define STACK_SIZE 2048
#define PIN_SEL  (1ULL<<DHT22PIN)

int numints = 0;
int64_t lastTs = 0;

struct flank
{
	bool val;
	uint32_t offset;
};
struct flank values[100];
int valueidx = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;

    int64_t ts = esp_timer_get_time();
	values[valueidx].offset = ts - lastTs;
	values[valueidx].val = gpio_get_level(DHT22PIN); // gpio.read();	
	lastTs =ts;
	valueidx++;
	numints++;

}
*/
extern "C" void app_main(void)
{
    dht22::Dht22Sensor sensor = dht22::Dht22Sensor((gpio_num_t)4);
    struct dht22::SensorReadingResult result;

    while(true)
    {
        sensor.read(&result);

        printf("Humidity:%.1f Temperature:%.1f ResultCode:%d\n",result.Humidity,result.Temperature,result.ResultCode);
	    vTaskDelay(5000 / portTICK_PERIOD_MS);        
    }


    /*
    esp_timer_early_init();

    gpio_install_isr_service( ESP_INTR_FLAG_LEVEL3);    
	lastTs = esp_timer_get_time();
    gpio_isr_handler_add(DHT22PIN, gpio_isr_handler, (void*)DHT22PIN);

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	printf("Dropping bus\n");

    io_conf.mode = GPIO_MODE_OUTPUT;
//    io_conf.pin_bit_mask = PIN_SEL;
//    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(DHT22PIN, 0);   
    ets_delay_us(18000);

//    gpio_intr_enable(DHT22PIN);
//    gpio_set_level(DHT22PIN, 1);
//    ets_delay_us(40);



    //hook isr handler for specific gpio pin
//	lastTs = esp_timer_get_time();
//    gpio_isr_handler_add(DHT22PIN, gpio_isr_handler, (void*)DHT22PIN);


//    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
//    io_conf.pin_bit_mask = PIN_SEL;
//    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1; // <== Hmm ...
    gpio_config(&io_conf);

    gpio_intr_enable(DHT22PIN);


    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    int lastValueIdx = 0;
	// Wait for data values from irq-handler
	while(true)
	{
        if(lastValueIdx != valueidx)
        {
            for(int i = lastValueIdx; i < valueidx;i++)
            {
                printf("%d",i);                
                printf(values[i].val?"+":"-");
                printf("%d\n",(int)values[i].offset);
            }
            lastValueIdx = valueidx;

        }
//		printf("Waiting for data (%d)\n",valueidx);
		vTaskDelay(1000 / portTICK_PERIOD_MS);		
	}
	
	printf("Data is ready!");	

	for(int i = 0; i < 82;i++)
	{
		printf(values[i].val?"+":"-");
		printf("%d\n",(int)values[i].offset);
	}
*/

}
