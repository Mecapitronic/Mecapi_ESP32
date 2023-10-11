#ifndef MODULE_H
#define MODULE_H

#include "ESP32_Helper.h"

// template <class T>
class IModule
{
   private:
    QueueHandle_t myQueue;
    int queueSize;

   public:
    // void Initialisation() = 0;
    virtual void CreateQueue(uint8_t queueType, int size)
    {
        queueSize = size;
        myQueue = xQueueCreate(queueSize, sizeof(uint8_t));
    }
    virtual void Config(uint8_t arg){
        // do something;
    };
};
/*
template <class T>
class Foo : public IModule<T>
{
   public:
    void functionA(){
        // do something;
    };
    void functionB(T arg){
        // do something;
    };
};
*/
#endif
