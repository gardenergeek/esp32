/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "dht22.h"
extern "C"
{
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
}

struct dhtTaskParameters
{
    gpio_num_t gpioNum;
    TaskHandle_t owner;
    struct SensorReadingResult *resultBuffer;
};

int64_t lastTs = 0;

struct IrqValue
{
	bool val;
	uint32_t time;
};

#define VALUEBUFFERSIZE 100

struct IrqState
{
    gpio_num_t PinNo;
    bool Collecting;
    int64_t LastTimerValue;
    struct IrqValue Values[VALUEBUFFERSIZE];
    int ValuesIdx;
    bool IsError;
};

struct IrqState irqState;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    IrqState *state = (IrqState *) arg;

    if(state->Collecting && !state->IsError)
    {
        int64_t ts = esp_timer_get_time();
        state->Values[state->ValuesIdx].time = ts - state->LastTimerValue;
        state->Values[state->ValuesIdx].val = gpio_get_level(state->PinNo);
        state->LastTimerValue = ts;

        state->ValuesIdx++;

        if(state->ValuesIdx > VALUEBUFFERSIZE)
        {
            state->IsError = true;
        }

    }
}

void init(gpio_num_t pin)
{
    // Init structure
    irqState.PinNo = pin;
    irqState.Collecting = false;
    irqState.LastTimerValue = 0;
    irqState.ValuesIdx = 0;
    irqState.IsError = false;

    gpio_install_isr_service( ESP_INTR_FLAG_LEVEL3);    
    gpio_isr_handler_add(irqState.PinNo, gpio_isr_handler, (void*)&irqState);

    // Reset pin, set as input & enable pullup
    gpio_reset_pin(irqState.PinNo);
    gpio_set_intr_type(irqState.PinNo,GPIO_INTR_ANYEDGE);
    gpio_set_pull_mode(irqState.PinNo,GPIO_PULLUP_ONLY);
    gpio_set_direction(irqState.PinNo,GPIO_MODE_INPUT);    
    gpio_intr_disable(irqState.PinNo);     
}
unsigned char timesToByte(int *start,int threshold)
{
	unsigned char result = 0;
	unsigned char mask = 0x0080;
	
	for(int i = 0; i < 8;i++)
	{
		if(start[i] > threshold)
		{
			result |= mask>>i;
		}
	}
	
	return result;
}

void readSensor(gpio_num_t pin,struct SensorReadingResult *result)
{
    // Reset state
    irqState.Collecting = true;
    irqState.LastTimerValue = esp_timer_get_time();
    irqState.ValuesIdx = 0;
    irqState.IsError = false;

    // Reset result
    result->Humidity = 0.0f;
    result->Temperature = 0.0f;
    result->ResultCode = Success;

    // Enable interuppts
    gpio_intr_enable(irqState.PinNo);

    // Drop bus ..
    gpio_set_direction(irqState.PinNo,GPIO_MODE_OUTPUT);
    gpio_pullup_dis(irqState.PinNo);
    gpio_set_level(irqState.PinNo, 0);  
    ets_delay_us(18000);

    // Sensor is awake, lets start reading
    gpio_set_direction(irqState.PinNo,GPIO_MODE_INPUT);
    gpio_pullup_en(irqState.PinNo);

    // Irqs should trigger, and start filling the buffer
    int rounds = 0;

    while(irqState.ValuesIdx < 86 && rounds < 10)
    {
	    vTaskDelay(500 / portTICK_PERIOD_MS);
        rounds++;
    }

    // Ok, we should be done, reset stuff
    gpio_intr_disable(irqState.PinNo); 
    irqState.Collecting = false;

    // Buffer overflow ?
    if(irqState.IsError)
    {
        result->ResultCode = BufferOverflow;
        return;
    }

    // Timeout ?
    if(rounds >= 10)
    {
        result->ResultCode= Timeout;
        return;
    }

    int bittimes[40];
	
	// Collect times to positive flanks,
	int bitidx = 0;
	for(int i = 0; i < 86;i++)
	{
        struct IrqValue v = irqState.Values[i];        
		// Ignore the first, its i ACK
		if(!v.val && i > 4)
		{
			bittimes[bitidx++] = v.time;
//			printf("%ld\n",v.time);			
		}
	}
    /*
	for(int i = 0; i < 40;i++)
	{
		printf("[%d] %d\n",i,bittimes[i]);			
	}
	*/
    int threshold = 50;
	unsigned char hhumidity = timesToByte(&bittimes[0],threshold);
	unsigned char lhumidity= timesToByte(&bittimes[8],threshold);	
	unsigned char htemp= timesToByte(&bittimes[16],threshold);
	unsigned char ltemp= timesToByte(&bittimes[24],threshold);	
	unsigned char check= timesToByte(&bittimes[32],threshold);
	
	unsigned char calculated = (hhumidity + lhumidity + htemp + ltemp) & 0x0FF;
	
	uint16_t humidity = hhumidity<<8 | lhumidity;
	uint16_t temp = htemp<<8 | ltemp;	
	
    temp = 0x8000 | temp;
    
//	printf("Humidity:%u Temp:%u\n",humidity,temp);				
//	printf("checksum:%u = %u\n",check,calculated);	

    // Checksum ?
    if(check != calculated)
    {
        result->ResultCode = CrcError;
        return;
    }
    else
    {
        if(temp & 0x8000)
        {
            temp = temp & 0x7fff;
            result->Temperature = -temp/10.0f;
        }
        else
        {
            result->Temperature = temp/10.0f;            
        }
        result->ResultCode = Success;
        result->Humidity = humidity/10.0f;

    }
}
void dht22Task( void * pvParameters )
{
    // Copy params to local vars & free
    dhtTaskParameters *params = (dhtTaskParameters *)pvParameters;

    gpio_num_t gpioNum = params->gpioNum;
    TaskHandle_t owner = params->owner;
    struct SensorReadingResult *resultBuffer = params->resultBuffer;

    delete params;

    uint32_t signalV;    

    gpio_num_t greenPinNo = (gpio_num_t)17;
    gpio_num_t redPinNo = (gpio_num_t)16;    

    gpio_reset_pin(greenPinNo);
    gpio_set_direction(greenPinNo, GPIO_MODE_OUTPUT);

    gpio_reset_pin(redPinNo);
    gpio_set_direction(redPinNo, GPIO_MODE_OUTPUT);

    init(gpioNum);
	vTaskDelay(5000 / portTICK_PERIOD_MS);
    while(true)
    {
        gpio_set_level(greenPinNo, 1);  
//        printf("Waiting for signal\n");        
        xTaskNotifyWait(0x00,ULONG_MAX,&signalV,portMAX_DELAY);

        // Ok, lets start to read
        gpio_set_level(greenPinNo, 0);
//        gpio_set_level(redPinNo, 1);
        
        readSensor(gpioNum,resultBuffer);
//        printf("Reading sensor done\n");

        xTaskNotify(owner,SIGNAL_READ,eNoAction);
  //      gpio_set_level(redPinNo, 0);
//        gpio_set_level(greenPinNo, 1);


    }
}


namespace dht22
{
    
    // gpioNum : 0..15
     Dht22Sensor::Dht22Sensor(uint32_t gpioNum)
        : m_gpioNum(gpioNum),m_taskHandle(NULL)
    {
        init();
    }
    void Dht22Sensor::init()
    {
        dhtTaskParameters *startParams = new dhtTaskParameters();
        
        startParams->gpioNum = (gpio_num_t)this->m_gpioNum;
        startParams->owner = xTaskGetCurrentTaskHandle();
        startParams->resultBuffer = &this->m_result;

        xTaskCreate( &dht22Task, "dhtTask", 2048, startParams, tskIDLE_PRIORITY, &this->m_taskHandle );
    }

    void Dht22Sensor::read(struct SensorReadingResult *result)
    {
        uint32_t signalV;
        // Lett task to read
        xTaskNotify(this->m_taskHandle,SIGNAL_READ,eNoAction);

        // Wait for result
        xTaskNotifyWait(0x00,ULONG_MAX,&signalV,portMAX_DELAY); 

        // Copy result
        result->Temperature = this->m_result.Temperature;
        result->Humidity = this->m_result.Humidity;
        result->ResultCode = this->m_result.ResultCode;

    }
}

