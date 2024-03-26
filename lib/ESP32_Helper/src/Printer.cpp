#include "Printer.h"

namespace Printer
{
    namespace
    {
        Level printLevel = LEVEL_VERBOSE;
        Enable printEnable = ENABLE_NONE;
    }  // namespace

    void PrintLevel(Level level)
    {
        printLevel = level;
        SERIAL_DEBUG.print("PrintLevel : ");
        switch (level)
        {
            ENUM_PRINT(LEVEL_VERBOSE);
            ENUM_PRINT(LEVEL_INFO);
            ENUM_PRINT(LEVEL_WARN);
            ENUM_PRINT(LEVEL_ERROR);
            ENUM_PRINT(LEVEL_NONE);
        }
    }
    Level PrintLevel() { return printLevel; }

    void PrintEnable(Enable enable)
    {
        if (enable != ENABLE_NONE)
        {
            printEnable = enable;
            SERIAL_DEBUG.print("PrintEnable : ");
            switch (enable)
            {
                ENUM_PRINT(ENABLE_FALSE);
                ENUM_PRINT(ENABLE_TRUE);
                ENUM_PRINT(ENABLE_NONE);
            }
        }
    }

    Enable PrintEnable() { return printEnable; }

    bool IsPrintable(Level level) { return PrintEnable() == ENABLE_TRUE && PrintLevel() <= level; }

    void println(Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.println();
    }

    void print(String prefix, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, int data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, int data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, data, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, uint data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, uint data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, data, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, char data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, char data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, data, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, float data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, float data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, data, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, String data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.print(suffix);
    }

    void println(String prefix, String data, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        print(prefix, data, suffix, level);
        SERIAL_DEBUG.println();
    }

    void print(String prefix, Point data, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
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

    void print(String prefix, Point3D data, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print("x: ");
        SERIAL_DEBUG.print(data.x);
        SERIAL_DEBUG.print(" y: ");
        SERIAL_DEBUG.print(data.y);
        SERIAL_DEBUG.print(" z: ");
        SERIAL_DEBUG.print(data.z);
        SERIAL_DEBUG.print(suffix);
        if (lineFeed)
            SERIAL_DEBUG.println();
    }

    void print(String prefix, Point4D data, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print("x: ");
        SERIAL_DEBUG.print(data.x);
        SERIAL_DEBUG.print(" y: ");
        SERIAL_DEBUG.print(data.y);
        SERIAL_DEBUG.print(" z: ");
        SERIAL_DEBUG.print(data.z);
        SERIAL_DEBUG.print(" w: ");
        SERIAL_DEBUG.print(data.w);
        SERIAL_DEBUG.print(suffix);
        if (lineFeed)
            SERIAL_DEBUG.println();
    }

    void print(String prefix, PolarPoint data, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print("A: ");
        SERIAL_DEBUG.print((int)(data.angle / 100));
        SERIAL_DEBUG.print(" D: ");
        SERIAL_DEBUG.print(data.distance);
        SERIAL_DEBUG.print(" C: ");
        SERIAL_DEBUG.print(data.confidence);
        SERIAL_DEBUG.print(" X: ");
        SERIAL_DEBUG.print(data.x);
        SERIAL_DEBUG.print(" Y: ");
        SERIAL_DEBUG.print(data.y);
        SERIAL_DEBUG.print(suffix);
        if (lineFeed)
            SERIAL_DEBUG.println();
    }

    void print(String prefix, Command cmd, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
            return;
        print("Command => " + prefix);
        print(" Cmd=" + cmd.cmd);
        print(" Size=", cmd.size);
        print(" Data=", cmd.data[0]);
        if (cmd.size > 1)
        {
            for (size_t size_data = 1; size_data < cmd.size; size_data++)
            {
                print(",", cmd.data[size_data]);
            }
        }
        if (lineFeed)
            SERIAL_DEBUG.println();
    }

    // bool needs to be the last because it overrides all functions
    void print(String prefix, bool data, String suffix, Level level, boolean lineFeed)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        if (data)
            SERIAL_DEBUG.print("true");
        else
            SERIAL_DEBUG.print("false");
        SERIAL_DEBUG.print(suffix);
        if (lineFeed)
            SERIAL_DEBUG.println();
    }

    void printArray(String prefix, int array[], size_t size, char separator, String suffix, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(prefix);
        if (size > 0)
        {
            for (size_t i = 0; i < size - 1; i++)
            {
                SERIAL_DEBUG.print(array[i]);
                SERIAL_DEBUG.print(separator);
            }
            SERIAL_DEBUG.print(array[size - 1]);
        }
        SERIAL_DEBUG.println(suffix);
    }

    void plotPoint(Point p, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        String data = ">" + varName + ":" + (int)p.x + ":" + (int)p.y + "|xy";
        SERIAL_DEBUG.println(data);
    }

    void plotPolarPoint(PolarPoint p, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        String data = ">" + varName + ":" + (int)p.x + ":" + (int)p.y + "|xy";
        SERIAL_DEBUG.println(data);
    }

    void plotPolarPoints(PolarPoint polarPoints[], uint16_t size, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        String data = ">" + varName + ":";
        for (uint16_t i = 0; i < size; i++)
        {
            data += String() + (int)(polarPoints[i].x) + ":" + (int)(polarPoints[i].y) + ";";
        }
        data += "|xy";
        SERIAL_DEBUG.println(data);
    }

    void plotPoint(Point p, Level level)
    {
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(">x:");
        SERIAL_DEBUG.println((int)p.x);
        SERIAL_DEBUG.print(">y:");
        SERIAL_DEBUG.println((int)p.y);
    }

    void plotScanTD(std::vector<PolarPoint> vec, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        if (vec.size() == 0)
            return;
        SERIAL_DEBUG.print(">" + varName + ":");
        for (uint16_t i = 0; i < vec.size(); i++)
        {
            SERIAL_DEBUG.print(String() + (int)vec[i].angle + ":" + (int)vec[i].distance + ";");
        }
        SERIAL_DEBUG.println("|td");
    }

    void plotScanXY(std::vector<PolarPoint> vec, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        if (vec.size() == 0)
            return;
        SERIAL_DEBUG.print(">" + varName + ":");
        for (uint16_t i = 0; i < vec.size(); i++)
        {
            SERIAL_DEBUG.print(String() + (int)vec[i].x + ":" + (int)vec[i].y + ";");
        }
        SERIAL_DEBUG.println("|xy");
    }

    void plotRobot(PolarPoint pos, Level level)
    {
        if (!IsPrintable(level))
            return;
        if (!IsPrintable(level))
            return;
        SERIAL_DEBUG.print(">robot:");
        SERIAL_DEBUG.print(String() + pos.x + ":" + pos.y + ";");
        SERIAL_DEBUG.println("|xy");
    }

    void plotTrackerPoint(PointTracker p, String varName, Level level)
    {
        if (!IsPrintable(level))
            return;
        String data = ">" + varName + ":" + (int)(p.point.x) + ":" + (int)(p.point.y);
        data += "|xy";
        SERIAL_DEBUG.println(data);
    }

    void plot3D(Point3D p, String varName)
    {
        // 3D|A:B:C|E
        // '3D|sphere1,widget0:S:sphere:RA:'+ str(sphere1rad)+':P:'+ str(sphere1x) +':'+ str(sphere1y) +':'+
        // str(sphere1z) + ':C:black:O:1' msg = '3D|sphere' + sphere1num + ',widget0:S:sphere:RA:' + str(sphere1rad) +
        // ':P:' + str(sphere1x) + ':' + str(sphere1y) + ':' + str(sphere1z)
        // + ':C:black:O:1' #2ecc71
        // TODO : rester en cm sinon le point est perdu dans le brouillard de l'horizon 3D de teleplot...
        // FIXME : la couleur n'est pas prise en compte
        // FIXME : le repère 3D de teleplot est basé sur un plan XZ avec Y en "hauteur" (sans axe!)
        String data =
            ">3D|" + varName + ",widget0:S:sphere:P:" + p.x + ":" + p.y + ":" + p.z / 10 + ":RA:1:C:black:O:1";
        SERIAL_DEBUG.println(data);
    }

    void plot3Dpy(Point3D p)
    {
        String data = "" + String(p.x) + ":" + String(p.y) + ":" + String(p.z);
        SERIAL_DEBUG.println(data);
    }
}  // namespace Printer
