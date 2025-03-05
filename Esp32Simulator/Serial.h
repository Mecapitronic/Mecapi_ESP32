#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "Arduino.h"

class MySerial
{
public:

	MySerial();
	~MySerial();

	void end();
	void setRxBufferSize(int size);
	void setTxBufferSize(int size);
	void begin(int baud_speed);
	void print();
	void print(const char* str);
	void print(String str);
	void print(int i);
	void println();
	void println(const char* str);
	void println(String str);
	void println(int i);
	int available();
	char read();
	void write(const char* str, int length);
};

extern MySerial Serial;

#endif