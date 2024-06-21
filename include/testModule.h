#ifndef MODULE
#define MODULE

#include "ModuleTemplate.h"
using namespace Printer;

class testModule : public IModule
{
   public:
    int test;
#pragma region IModule
    testModule(void);
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void SetConfig(int t);
#pragma endregion
};
#endif
