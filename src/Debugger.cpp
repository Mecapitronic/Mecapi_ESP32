#include "Debugger.h"

QueueHandle_t queueSteps;
const uint16_t queueStepsSize = 100;
uint16_t debugStep;

void Debugger::init()
{
    if (enabled)
    {
        SERIAL_DEBUG.print("Preparing queueSteps : ");
        queueSteps = xQueueCreate(queueStepsSize, sizeof(uint16_t));
        if (queueSteps == NULL)
        {
            SERIAL_DEBUG.println("Error creating the queueSteps");
        }
        debugStep = 0;
        delay(200);
        SERIAL_DEBUG.println("done.");
    }
    else
        SERIAL_DEBUG.println("Debugger Disable");
}

bool Debugger::waitForAvailableSteps()
{
    if (debugStep == 0)
        SERIAL_DEBUG.println("waitForAvailableSteps");

    while (debugStep == 0)
    {
        if (uxQueueMessagesWaiting(queueSteps) > 0)
        {
            uint16_t steps = 0;
            if (xQueueReceive(queueSteps, &steps, portTICK_PERIOD_MS * 0))
            {
                addSteps(steps);
                SERIAL_DEBUG.print("xQueueReceive queueSteps : ");
                SERIAL_DEBUG.println(steps);
            }
        }
        vTaskDelay(1);
    }
    subSteps();
    return true;
}

void Debugger::addSteps(uint16_t steps) { debugStep += steps; }

void Debugger::subSteps(uint16_t steps) { debugStep -= steps; }
