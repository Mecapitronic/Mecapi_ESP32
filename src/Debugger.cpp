#include "Debugger.h"

namespace Debugger
{
namespace  // anonymous nested namespace, cannot access outside this file
{
QueueHandle_t queueSteps = nullptr;
const uint16_t queueStepsSize = 100;
Enable debuggerEnable = ENABLE_NONE;
}  // namespace

void EnableDebugger(Enable enable)
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
        }
        if (debuggerEnable != enable && enable == ENABLE_FALSE)
        {
            Printer::print(" Disable");
        }
        Printer::println();
        debuggerEnable = enable;
    }
}

bool IsEnable() { return debuggerEnable == ENABLE_TRUE; }

void Initialisation()
{
    Printer::print("Debugger : ");
    Printer::print("Preparing queueSteps : ");
    queueSteps = xQueueCreate(queueStepsSize, sizeof(int16_t));
    if (queueSteps == NULL)
    {
        Printer::println("Error creating the queueSteps !");
    }
    Printer::println("done.");
}

bool WaitForAvailableSteps()
{
    static int16_t debugStep = 0;
    if (IsEnable())
    {
        if (debugStep <= 0)
            Printer::println("waitForAvailableSteps");

        while (debugStep <= 0 && IsEnable())
        {
            if (uxQueueMessagesWaiting(queueSteps) > 0)
            {
                int16_t steps = 0;
                if (xQueueReceive(queueSteps, &steps, portTICK_PERIOD_MS * 0))
                {
                    debugStep += steps;
                    Printer::println("xQueueReceive queueSteps : ", debugStep);
                }
            }
            vTaskDelay(1);
        }
        debugStep--;
    }
    else if (debugStep != 0)
        debugStep = 0;
    return true;
}

    void AddSteps(int16_t steps)
    {
        xQueueSend(queueSteps, &steps, 0);
        Printer::println("Adding ", steps, " steps to Debugger.");
    }

}  // namespace Debugger
