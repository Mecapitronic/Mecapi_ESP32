#ifndef SERIAL_H
#define SERIAL_H

#include "Arduino.h"
#include "WString.h"
#include <iostream>

using namespace std;

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