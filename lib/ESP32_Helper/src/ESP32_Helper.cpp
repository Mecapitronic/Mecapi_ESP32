#include "ESP32_Helper.h"

using namespace Printer;

namespace ESP32_Helper
{
namespace
{
uint16_t indexBuffer = 0;
uint16_t indexSeparator = 0;
const int Serial_Read_Buffer = 64;
char readBuffer[Serial_Read_Buffer];
Command cmdTmp;
char tmpChar = '\0';
std::queue<Command> awaitingCommand;

void resetVar()
{
    indexBuffer = 0;
    indexSeparator = 0;
    cmdTmp.cmd = "";
    cmdTmp.size = -1;
    for (size_t i = 0; i < 8; i++)
    {
        cmdTmp.data[i] = 0;
    }
}
}  // namespace

void ESP32_Helper(int baud_speed)
{
    SERIAL_DEBUG.begin(baud_speed);
    if (SERIAL_DEBUG.available() > 0)
    {
        SERIAL_DEBUG.flush();
    }
    SERIAL_DEBUG.println();

    Printer::PrintEnable(Enable::ENABLE_TRUE);
    Printer::PrintLevel(Level::LEVEL_VERBOSE);

    println();
    println("|--------------|");
    println("  MECAPITRONIC  ");
    println("|--------------|");
    println();
    print(__DATE__);
    print(" at ");
    println(__TIME__);
    println();

    Debugger::EnableDebugger(ENABLE_TRUE);
    Debugger::Initialisation();

    resetVar();
}

void UpdateSerial()
{
    while (SERIAL_DEBUG.available() > 0)
    {
        tmpChar = SERIAL_DEBUG.read();
        if (indexBuffer < Serial_Read_Buffer)
        {
            readBuffer[indexBuffer++] = tmpChar;

            if (tmpChar == ':')
            {
                if (cmdTmp.cmd == "" && indexBuffer > 1)
                {
                    cmdTmp.cmd = String(&readBuffer[0], indexBuffer - 1);
                    indexSeparator = indexBuffer;
                }
            }
            if (tmpChar == '\n')
            {
                SERIAL_DEBUG.print("Received : ");
                SERIAL_DEBUG.write(readBuffer, indexBuffer);

                int32_t index1 = indexSeparator;
                cmdTmp.size = 0;

                for (uint8_t i = indexSeparator; i < indexBuffer; i++)
                {
                    if (readBuffer[i] == ';' || readBuffer[i] == ':' || readBuffer[i] == '\n')
                    {
                        cmdTmp.data[cmdTmp.size++] = atoi(String(&readBuffer[index1], i - index1).c_str());
                        index1 = i + 1;
                    }
                }

                print("Received", cmdTmp);

                // We first handle if the command is for the Lib
                if (cmdTmp.cmd == "DebugSteps")
                {
                    // DebugSteps:10
                    Debugger::AddSteps(cmdTmp.data[0]);
                }
                else if (cmdTmp.cmd == "DebugEnable")
                {
                    // DebugSteps:1
                    Debugger::EnableDebugger((Enable)cmdTmp.data[0]);
                }
                else if (cmdTmp.cmd == "PrintLevel")
                {
                    // PrintLevel:0
                    Printer::PrintLevel((Level)cmdTmp.data[0]);
                }
                else if (cmdTmp.cmd == "PrintEnable")
                {
                    // PrintEnable:1
                    Printer::PrintEnable((Enable)cmdTmp.data[0]);
                }
                else
                {
                    // If command is not for Lib, we sent it to the main
                    awaitingCommand.push(cmdTmp);
                }
                resetVar();
            }
        }
        else
        {
            SERIAL_DEBUG.print("Read Buffer Overflow : ");
            SERIAL_DEBUG.println(indexBuffer);
            SERIAL_DEBUG.flush();
            resetVar();
        }
    }
}

bool HasWaitingCommand() { return awaitingCommand.size() > 0; }

Command GetCommand()
{
    Command cmd;
    cmd.cmd = "";
    if (HasWaitingCommand())
    {
        cmd = awaitingCommand.front();
        awaitingCommand.pop();
        print("POP", cmd);
    }
    return cmd;
}

}  // namespace ESP32_Helper
