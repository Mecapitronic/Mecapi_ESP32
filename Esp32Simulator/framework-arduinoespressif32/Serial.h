#ifndef SERIAL_H
#define SERIAL_H

#include "Arduino.h"
#include "WString.h"
#include <iostream>

using namespace std;

enum SerialConfig {
	SERIAL_5N1 = 0x8000010,
	SERIAL_6N1 = 0x8000014,
	SERIAL_7N1 = 0x8000018,
	SERIAL_8N1 = 0x800001c,
	SERIAL_5N2 = 0x8000030,
	SERIAL_6N2 = 0x8000034,
	SERIAL_7N2 = 0x8000038,
	SERIAL_8N2 = 0x800003c,
	SERIAL_5E1 = 0x8000012,
	SERIAL_6E1 = 0x8000016,
	SERIAL_7E1 = 0x800001a,
	SERIAL_8E1 = 0x800001e,
	SERIAL_5E2 = 0x8000032,
	SERIAL_6E2 = 0x8000036,
	SERIAL_7E2 = 0x800003a,
	SERIAL_8E2 = 0x800003e,
	SERIAL_5O1 = 0x8000013,
	SERIAL_6O1 = 0x8000017,
	SERIAL_7O1 = 0x800001b,
	SERIAL_8O1 = 0x800001f,
	SERIAL_5O2 = 0x8000033,
	SERIAL_6O2 = 0x8000037,
	SERIAL_7O2 = 0x800003b,
	SERIAL_8O2 = 0x800003f
};

#ifdef _WINDLL
//when in dll mode, we print over the OutputDebugString
void __cdecl myprintf(string str)
{
	OutputDebugString(str.c_str());
}
void __cdecl myprintf(const char* format, ...)
{
	char    buf[4096], * p = buf;
	va_list args;
	int     n;

	va_start(args, format);
	n = _vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
	va_end(args);

	p += (n < 0) ? sizeof buf - 3 : n;

	while (p > buf && isspace(p[-1]))
		*--p = '\0';

	*p++ = '\r';
	*p++ = '\n';
	*p = '\0';

	OutputDebugString(buf);
}

#else
// When in Console mode, we print over it
#define myprintf cout<<
#endif

class MySerial
{
public:
	int bytes = 0;
	string incoming = "";

	MySerial();
	~MySerial();

	void end();
	void flush();
	void setRxBufferSize(int size);
	void setTxBufferSize(int size);
	void begin(int baud_speed);
	void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);
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
	void write(int n);
};

extern MySerial Serial;
extern MySerial Serial1;
extern MySerial Serial2;

#endif