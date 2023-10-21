#ifndef MODULE_H
#define MODULE_H

#include "ESP32_Helper.h"

// using namespace Printer;

// template <class T>
class IModule
{
   private:
    QueueHandle_t myQueue;
    static const int queueSize = 500;

   public:
    virtual void Initialisation();
    virtual void SetConfig();
    virtual bool ReadSerial();
    virtual void SendSerial();
    virtual void AnalyseSerial();
    virtual void CreateQueue(int size = queueSize)
    {
        myQueue = xQueueCreate(size, sizeof(uint8_t));
        if (myQueue == NULL)
        {
            //Printer::println("Error creating the queue", LEVEL_ERROR);
        }
    };
};
#endif
