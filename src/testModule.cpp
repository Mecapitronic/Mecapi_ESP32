#include "testModule.h"

#pragma region IModule

testModule::testModule() { println("Constructor testModule"); }
void testModule::Initialisation() { println("Init testModule"); }
void testModule::Update() { println("Update testModule"); }
void testModule::HandleCommand(Command cmd) { print("testModule : ", cmd); }

#pragma endregion

void testModule::SetConfig(int t)
{
    test = t;
    println("SetConfig testModule");
}
