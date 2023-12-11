#ifndef MODULE
#define MODULE

#include "ModuleTemplate.h"
using namespace Printer;

class testModule : public IModule
{
   public:
#pragma region IModule
    testModule(void);
    // ~testModule();
    // void Initialisation();
    bool ReadSerial();
    void SendSerial();
    void HandleCommand(Command cmd);
#pragma endregion

    void SetConfig();
    void Analyse();
};
#endif
