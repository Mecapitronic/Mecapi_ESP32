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
    IModule(void)
    {
        CreateQueue();
        return;
    };
    virtual ~IModule(){}
    virtual void Initialisation()=0;
    virtual void SetConfig() = 0;
    virtual bool ReadSerial() = 0;
    virtual void SendSerial() = 0;
    virtual void Analyse() = 0;
    virtual void HandleCommand(Command cmd) = 0;

    void CreateQueue(int size = queueSize)
    {
        myQueue = xQueueCreate(size, sizeof(uint8_t));
        if (myQueue == NULL)
        {
            //Printer::println("Error creating the queue", LEVEL_ERROR);
        }
    };
};
#endif
