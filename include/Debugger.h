/**
 * @file Debugger.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief Software debugger
 * @date 2023-07-25
 */
#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "ESP32_Helper.h"

namespace Debugger
{

Enable EnableDebugger(Enable enable);
bool IsEnable();

void Initialisation();
bool WaitForAvailableSteps();
void AddSteps(uint16_t steps);
void SubSteps(uint16_t steps = 1);
};  // namespace Debugger
#endif
