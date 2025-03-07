#ifndef FREE_RTOS_H
#define FREE_RTOS_H

#include "Arduino.h"
#include <vector>

/*------------------------------*/
#define portTICK_PERIOD_MS 1
#define portBASE_TYPE               int
typedef portBASE_TYPE               BaseType_t;
typedef unsigned portBASE_TYPE      UBaseType_t;
typedef uint32_t TickType_t;
/*------------------------------*/


/*------------------------------*/
//using std::vector;

#define QueueHandle_t vector<BaseType_t>*
//typedef vector<BaseType_t> QueueHandle_t;
/*------------------------------*/


/*------------------------------*/
QueueHandle_t xQueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize);
void xQueueSend(QueueHandle_t xQueue, * pvItemToQueue, TickType_t xTicksToWait);
UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void* const pvBuffer, TickType_t xTicksToWait);
/*------------------------------*/

#endif