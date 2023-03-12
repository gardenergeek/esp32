#ifndef _DHT22_H_
#define _DHT22_H_

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SIGNAL_READ 1

enum SensorReadingResultCode
{
    Success,
    BufferOverflow,
    Timeout,
	CrcError
};

struct SensorReadingResult
{
	int16_t Humidity;
	int16_t Temperature;
	enum SensorReadingResultCode ResultCode;
};

namespace dht22
{	
	class Dht22Sensor
	{
	public:
		Dht22Sensor(uint32_t gpioNum);
		void read(struct SensorReadingResult *result);
		
	private:
		void init();

	private:
		uint8_t  m_gpioNum;
		TaskHandle_t m_taskHandle;
		struct SensorReadingResult m_result;
	};
}

#endif


