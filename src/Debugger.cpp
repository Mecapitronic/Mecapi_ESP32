#include "Debugger.h"

char readBuffer[READ_SERIAL_BUFFER_SIZE];
uint16_t indexBuffer;

QueueHandle_t queueSteps;
const uint16_t queueStepsSize = 100;
uint16_t debugStep;

void Debugger::init()
{
    if (enabled)
    {
        SERIAL_DEBUG.begin(SERIAL_DEBUG_SPEED);
        if (SERIAL_DEBUG.available() > 0)
        {
            SERIAL_DEBUG.flush();
        }

        strcpy(readBuffer, "");
        indexBuffer = 0;

        header();
        SERIAL_DEBUG.print("Preparing system...");
        queueSteps = xQueueCreate(queueStepsSize, sizeof(uint16_t));
        if (queueSteps == NULL)
        {
            log("Error creating the queueSteps", ERROR);
        }
        debugStep = 0;
        delay(200);
        SERIAL_DEBUG.println("done.");
    }
    else
        SERIAL_DEBUG.print("Debugger Disable");
}

void Debugger::header()
{
    SERIAL_DEBUG.println();
    SERIAL_DEBUG.println(".--------------.");
    SERIAL_DEBUG.println("  MECAPITRONIC  ");
    SERIAL_DEBUG.println("'--------------'");
    SERIAL_DEBUG.println();
    SERIAL_DEBUG.print(__DATE__);
    SERIAL_DEBUG.print(" at ");
    SERIAL_DEBUG.println(__TIME__);
    SERIAL_DEBUG.println();
}

Level Debugger::level() { return debugLevel; }

char* Debugger::checkSerial()
{
    if (enabled && SERIAL_DEBUG.available() > 0)
    {
        char tmpChar = SERIAL_DEBUG.read();
        if (indexBuffer < READ_SERIAL_BUFFER_SIZE)
        {
            readBuffer[indexBuffer++] = tmpChar;
            if (tmpChar == '\n')
            {
                SERIAL_DEBUG.print("Received : ");
                SERIAL_DEBUG.write(readBuffer, indexBuffer);
                indexBuffer = 0;
                return readBuffer;
            }
        }
        else
        {
            SERIAL_DEBUG.print("Read Buffer Overflow : ");
            SERIAL_DEBUG.println(indexBuffer);
            indexBuffer = 0;
        }
    }
    return nullptr;
}

void Debugger::log(String data, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(data);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, int data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, char data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, float data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, Point data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print("x: ");
    SERIAL_DEBUG.print(data.x);
    SERIAL_DEBUG.print(" y: ");
    SERIAL_DEBUG.print(data.y);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, PolarPoint data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print("A: ");
    SERIAL_DEBUG.print((int)(data.angle / 100));
    SERIAL_DEBUG.print(" D: ");
    SERIAL_DEBUG.print(data.distance);
    SERIAL_DEBUG.print(" C: ");
    SERIAL_DEBUG.println(data.confidence);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

// bool needs to be the last because it overrides all functions
void Debugger::log(String prefix, bool data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::logArray(String prefix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel)
        return;
    if (size > 0)
    {
        SERIAL_DEBUG.print(prefix);
        for (size_t i = 0; i < size - 1; i++)
        {
            SERIAL_DEBUG.print(array[i]);
            SERIAL_DEBUG.print(separator);
        }
        SERIAL_DEBUG.print(array[size - 1]);
        SERIAL_DEBUG.println(suffix);
    }
}

void Debugger::logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel)
        return;
    if (size > 0)
    {
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(element);
        SERIAL_DEBUG.print(interFix);
        for (size_t i = 0; i < size - 1; i++)
        {
            SERIAL_DEBUG.print(array[i]);
            SERIAL_DEBUG.print(separator);
        }
        SERIAL_DEBUG.print(array[size - 1]);
        SERIAL_DEBUG.println(suffix);
    }
    else
        SERIAL_DEBUG.println("Invalid array printed !");
}

void Debugger::plotPoint(Point p, String varName, Level level)
{
    if (level < debugLevel)
        return;
    String data = ">" + varName + ":" + (int)p.x + ":" + (int)p.y + "|xy";
    SERIAL_DEBUG.println(data);
}

void Debugger::plotPoint(Point p, Level level)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(">x:");
    SERIAL_DEBUG.println((int)p.x);
    SERIAL_DEBUG.print(">y:");
    SERIAL_DEBUG.println((int)p.y);
}

void Debugger::plot3D(Point3D p, String varName)
{
    // 3D|A:B:C|E
    // '3D|sphere1,widget0:S:sphere:RA:'+ str(sphere1rad)+':P:'+ str(sphere1x) +':'+ str(sphere1y) +':'+ str(sphere1z) + ':C:black:O:1'
    // msg = '3D|sphere' + sphere1num + ',widget0:S:sphere:RA:' + str(sphere1rad) + ':P:' + str(sphere1x) + ':' + str(sphere1y) + ':' + str(sphere1z)
    // + ':C:black:O:1' #2ecc71
    // TODO : rester en cm sinon le point est perdu dans le brouillard de l'horizon 3D de teleplot...
    // FIXME : la couleur n'est pas prise en compte
    // FIXME : le repère 3D de teleplot est basé sur un plan XZ avec Y en "hauteur" (sans axe!)
    String data = ">3D|" + varName + ",widget0:S:sphere:P:" + p.x + ":" + p.y + ":" + p.z / 10 + ":RA:1:C:black:O:1";
    SERIAL_DEBUG.println(data);
}

void Debugger::plot3Dpy(Point3D p)
{
    String data = "" + String(p.x) + ":" + String(p.y) + ":" + String(p.z);
    SERIAL_DEBUG.println(data);
}

bool Debugger::waitForAvaiableSteps()
{
    if (debugStep == 0)
        SERIAL_DEBUG.println("waitForAvaiableSteps");

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
