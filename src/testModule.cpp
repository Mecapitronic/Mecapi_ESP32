#include "testModule.h"

#pragma region IModule
testModule::testModule()
{
    println("Constructor testModule");
    return;
}
// testModule::~testModule() {}
// void testModule::Initialisation() { println("Init testModule"); }
bool testModule::ReadSerial() { return true; }
void testModule::SendSerial() {}
void testModule::HandleCommand(Command cmd) { print("testModule : ", cmd); }

#pragma endregion

void testModule::SetConfig() {}
void testModule::Analyse() {}
