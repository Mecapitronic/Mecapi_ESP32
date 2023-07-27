#include "Debugger.h"

namespace Debugger
{
namespace  // anonymous nested namespace, cannot access outside this file
{
QueueHandle_t queueSteps = nullptr;
const uint16_t queueStepsSize = 100;
uint16_t debugStep = 0;
Enable debuggerEnable = ENABLE_NONE;
}  // namespace

Enable EnableDebugger(Enable enable)
{
    if (enable != ENABLE_NONE)
    {
        Printer::print("Debugger : ");

        if (debuggerEnable == enable && enable == ENABLE_TRUE)
        {
            Printer::print("already Enable");
        }
        if (debuggerEnable == enable && enable == ENABLE_FALSE)
        {
            Printer::print("already Disable");
        }
        if (debuggerEnable != enable && enable == ENABLE_TRUE)
        {
            Printer::print(" Enable");
            debugStep = 0;
        }
        if (debuggerEnable != enable && enable == ENABLE_FALSE)
        {
            Printer::print(" Disable");
            debugStep = 1;  // so we can escape the current waiting loop
        }
        debuggerEnable = enable;
    }
    return debuggerEnable;
}

bool IsEnable() { return debuggerEnable == ENABLE_TRUE; }

void Initialisation()
{
    Printer::print("Debugger : ");
    Printer::print("Preparing queueSteps : ");
    queueSteps = xQueueCreate(queueStepsSize, sizeof(uint16_t));
    if (queueSteps == NULL)
    {
        Printer::println("Error creating the queueSteps !");
    }
    debugStep = 0;
    Printer::println("done.");
}

bool WaitForAvailableSteps()
{
    if (IsEnable())
    {
        if (debugStep == 0)
            Printer::println("waitForAvailableSteps");

        while (debugStep == 0)
        {
            if (uxQueueMessagesWaiting(queueSteps) > 0)
            {
                uint16_t steps = 0;
                if (xQueueReceive(queueSteps, &steps, portTICK_PERIOD_MS * 0))
                {
                    AddSteps(steps);
                    Printer::print("xQueueReceive queueSteps : ", steps);
                }
            }
            vTaskDelay(1);
        }
        SubSteps();
    }
    return true;
}

void AddSteps(uint16_t steps) { debugStep += steps; }

void SubSteps(uint16_t steps) { debugStep -= steps; }
}  // namespace Debugger
