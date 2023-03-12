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

namespace dht22
{
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
    bool Dht22Sensor::IsCommonInitialized = false;
    // gpioNum : 0..15
     Dht22Sensor::Dht22Sensor(gpio_num_t gpioNum)
    {
        // Init structure
        this->m_irqState.PinNo = gpioNum;
        this->m_irqState.Collecting = false;
        this->m_irqState.LastTimerValue = 0;
        this->m_irqState.ValuesIdx = 0;
        this->m_irqState.IsError = false;

        init();
    }
    void Dht22Sensor::init()
    {
        if(!Dht22Sensor::IsCommonInitialized)
        {
            gpio_install_isr_service( ESP_INTR_FLAG_LEVEL3);    
            Dht22Sensor::IsCommonInitialized = true;
        }

        gpio_isr_handler_add(this->m_irqState.PinNo, gpio_isr_handler, (void*)&this->m_irqState);

        // Reset pin, set as input & enable pullup
        gpio_reset_pin(this->m_irqState.PinNo);
        gpio_set_intr_type(this->m_irqState.PinNo,GPIO_INTR_ANYEDGE);
        gpio_set_pull_mode(this->m_irqState.PinNo,GPIO_PULLUP_ONLY);
        gpio_set_direction(this->m_irqState.PinNo,GPIO_MODE_INPUT);    
        gpio_intr_disable(this->m_irqState.PinNo);     

    }

    void Dht22Sensor::read(struct SensorReadingResult *result)
    {
        // Reset state
        this->m_irqState.Collecting = true;
        this->m_irqState.LastTimerValue = esp_timer_get_time();
        this->m_irqState.ValuesIdx = 0;
        this->m_irqState.IsError = false;

        // Reset result
        result->Humidity = 0.0f;
        result->Temperature = 0.0f;
        result->ResultCode = Success;

        // Enable interuppts
        gpio_intr_enable(this->m_irqState.PinNo);

        // Drop bus ..
        gpio_set_direction(this->m_irqState.PinNo,GPIO_MODE_OUTPUT);
        gpio_pullup_dis(this->m_irqState.PinNo);
        gpio_set_level(this->m_irqState.PinNo, 0);  
        ets_delay_us(18000);

        // Sensor is awake, lets start reading
        gpio_set_direction(this->m_irqState.PinNo,GPIO_MODE_INPUT);
        gpio_pullup_en(this->m_irqState.PinNo);

        // Irqs should trigger, and start filling the buffer
        int rounds = 0;

        while(this->m_irqState.ValuesIdx < 86 && rounds < 10)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            rounds++;
        }

        // Ok, we should be done, reset stuff
        gpio_intr_disable(this->m_irqState.PinNo); 
        this->m_irqState.Collecting = false;

        // Buffer overflow ?
        if(this->m_irqState.IsError)
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
            struct IrqValue v = this->m_irqState.Values[i];        
            // Ignore the first, its i ACK
            if(!v.val && i > 4)
            {
                bittimes[bitidx++] = v.time;
    //			printf("%ld\n",v.time);			
            }
        }

        int threshold = 50;
        unsigned char hhumidity = timesToByte(&bittimes[0],threshold);
        unsigned char lhumidity= timesToByte(&bittimes[8],threshold);	
        unsigned char htemp= timesToByte(&bittimes[16],threshold);
        unsigned char ltemp= timesToByte(&bittimes[24],threshold);	
        unsigned char check= timesToByte(&bittimes[32],threshold);
        
        unsigned char calculated = (hhumidity + lhumidity + htemp + ltemp) & 0x0FF;
        
        uint16_t humidity = hhumidity<<8 | lhumidity;
        uint16_t temp = htemp<<8 | ltemp;	

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
    unsigned char Dht22Sensor::timesToByte(int *start,int threshold)
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

}

