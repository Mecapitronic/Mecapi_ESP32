#include "FreeRTOS.h"

void vTaskDelay(int milli)
{
}

QueueHandle_t xQueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize)
{
	return QueueHandle_t();
}

void xQueueSend(QueueHandle_t xQueue, void * pvItemToQueue, TickType_t xTicksToWait)
{
}

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue)
{
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void* const pvBuffer, TickType_t xTicksToWait)
{
}

/*------------------------------*/
