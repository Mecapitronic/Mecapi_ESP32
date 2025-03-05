/********** INCLUDE *********/
#include "Serial.h"

MySerial::MySerial() {};
MySerial::~MySerial() {};

void MySerial::end() {};
void MySerial::setRxBufferSize(int size) {};
void MySerial::setTxBufferSize(int size) {};
void MySerial::begin(int baud_speed) {};
void MySerial::print() {};
void MySerial::print(const char* str) {};
void MySerial::print(String str) {};
void MySerial::print(int i) {};
void MySerial::println() {};
void MySerial::println(const char* str) {};
void MySerial::println(String str) {};
void MySerial::println(int i) {};
int MySerial::available() { return 0; };
char MySerial::read() { return ' '; };
void MySerial::write(const char* str, int length) {};

MySerial Serial = MySerial();