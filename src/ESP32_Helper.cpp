#include "ESP32_Helper.h"

using namespace Printer;

namespace ESP32_Helper
{

namespace
{
uint16_t indexBuffer = 0;
const int Serial_Read_Buffer = 64;
char readBuffer[Serial_Read_Buffer];
bool foundFirstColon = false;
int indexFirstColon = 0;
bool foundSecondColon = false;
int indexSecondColon = 0;
char tmpChar = '\0';
std::queue<Command> awaitingCommand;

void resetVar()
{
    indexBuffer = 0;
    foundFirstColon = false;
    indexFirstColon = 0;
    foundSecondColon = false;
    indexSecondColon = 0;
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
}

void UpdateSerial()
{
    while (SERIAL_DEBUG.available() > 0)
    {
        tmpChar = SERIAL_DEBUG.read();
        if (indexBuffer < Serial_Read_Buffer)
        {
            readBuffer[indexBuffer++] = tmpChar;

            if (foundFirstColon && !foundSecondColon && tmpChar == ':')
            {
                foundSecondColon = true;
                indexSecondColon = indexBuffer;
            }
            if (!foundFirstColon && tmpChar == ':')
            {
                foundFirstColon = true;
                indexFirstColon = indexBuffer;
            }
            if (tmpChar == '\n')
            {
                SERIAL_DEBUG.print("Received : ");
                SERIAL_DEBUG.write(readBuffer, indexBuffer);
                if (foundFirstColon && foundSecondColon)
                {
                    Command cmd = {String(&readBuffer[0], indexFirstColon - 1), String(&readBuffer[indexFirstColon], indexSecondColon - indexFirstColon - 1),
                                   atoi(String(&readBuffer[indexSecondColon], indexBuffer - indexSecondColon).c_str())};
                    print("Received", cmdTmp);
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
        cmd.cat = "BAD";
    if (HasWaitingCommand())
    {
        cmd = awaitingCommand.front();
        awaitingCommand.pop();
            print("POP", cmd);
    }
    return cmd;
}

}  // namespace ESP32_Helper
