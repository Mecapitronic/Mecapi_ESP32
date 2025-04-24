/********** INCLUDE *********/
#include "Serial.h"

MySerial::MySerial() {};
MySerial::~MySerial() {};

void MySerial::end() {};
void MySerial::flush() {};
void MySerial::setRxBufferSize(int size) {};
void MySerial::setTxBufferSize(int size) {};
void MySerial::begin(int baud_speed) {};
void MySerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {};
void MySerial::print() {};
void MySerial::print(const char* str) { if(str != NULL) myprintf(str); };
void MySerial::print(String str) {
	if (str != "")
	{
		string s = str.c_str();
		myprintf(s);
	}
};
void MySerial::print(int i) { if (i != NULL) myprintf(i); };
void MySerial::println() { myprintf('\n'); };
void MySerial::println(const char* str) { print(str); println(); };
void MySerial::println(String str) { print(str); println(); };
void MySerial::println(int i) { print(i); println(); };
int MySerial::available() { return bytes; };

char MySerial::read()
{
	char c = incoming[0];
	incoming.erase(0,1);
	bytes = incoming.length();
	return c;
};
void MySerial::write(const char* str, int length)
{
	for (int i = 0; i < length; i++)
	{
		char c = str[i];
		myprintf(c);
	} 
};

void MySerial::write(int n)
{
	myprintf(n);
}

MySerial Serial = MySerial();
MySerial Serial1 = MySerial();
MySerial Serial2 = MySerial();