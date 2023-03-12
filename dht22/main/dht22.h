#ifndef _DHT22_H_
#define _DHT22_H_

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define SIGNAL_READ 1
#define VALUEBUFFERSIZE 100
namespace dht22
{	
	struct IrqValue
	{
		bool val;
		uint32_t time;
	};

	struct IrqState
	{
		gpio_num_t PinNo;
		bool Collecting;
		int64_t LastTimerValue;
		struct IrqValue Values[VALUEBUFFERSIZE];
		int ValuesIdx;
		bool IsError;
	};


	enum SensorReadingResultCode
	{
		Success,
		BufferOverflow,
		Timeout,
		CrcError
	};

	struct SensorReadingResult
	{
		float Humidity;
		float Temperature;
		enum SensorReadingResultCode ResultCode;
	};

	class Dht22Sensor
	{
	public:
		Dht22Sensor(gpio_num_t gpioNum);
		void read(struct SensorReadingResult *result);
		
	private:
		void init();
		unsigned char timesToByte(int *start,int threshold);

	private:
		struct SensorReadingResult m_result;
		struct IrqState m_irqState;
	};
}

#endif


