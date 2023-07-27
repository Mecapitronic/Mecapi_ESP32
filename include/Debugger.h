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

void EnableDebugger(Enable enable);
bool IsEnable();

void Initialisation();
bool WaitForAvailableSteps();
void AddSteps(int16_t steps);
};  // namespace Debugger
#endif
