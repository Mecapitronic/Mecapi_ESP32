#include "testModule.h"

testModule::testModule(void) { return; }
// testModule ::~testModule() {}
void testModule::Initialisation() {}
void testModule::SetConfig() {}
bool testModule::ReadSerial() { return true; }
void testModule::SendSerial() {}
void testModule::Analyse() {}
void testModule::HandleCommand(Command cmd) { print("testModule : ", cmd); }

// void testModule::CreateQueue(int size = 0) {}
