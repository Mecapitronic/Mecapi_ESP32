#ifndef Arduino_h
#define Arduino_h

#include "WString.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

//Interrupt Modes
#define RISING    0x01
#define FALLING   0x02
#define CHANGE    0x03
#define ONLOW     0x04
#define ONHIGH    0x05
#define ONLOW_WE  0x0C
#define ONHIGH_WE 0x0D

#define DEFAULT 1
#define EXTERNAL 0

#ifndef __STRINGIFY
#define __STRINGIFY(a) #a
#endif

// can't define max() / min() because of conflicts with C++
#define _min(a,b) ((a)<(b)?(a):(b))  
#define _max(a,b) ((a)>(b)?(a):(b))
#define _abs(x) ((x)>0?(x):-(x))  // abs() comes from STL
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  // round() comes from STL
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

/* Visual studio intrinsics used so the __debugbreak() function is available
 * should an assert get hit. */
#include <intrin.h>

#include "FreeRTOS.h"

#define millis() (uint64_t) (xTaskGetTickCount() * portTICK_PERIOD_MS)

// Get time stamp in milliseconds.
/*uint64_t millis()
{
	uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now().time_since_epoch())
		.count();
	return ms;
}*/

// Get time stamp in microseconds.
/*uint64_t micros()
{
	uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::high_resolution_clock::now().time_since_epoch())
		.count();
	return us;
}*/

// Get time stamp in nanoseconds.
/*uint64_t nanos()
{
	uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::high_resolution_clock::now().time_since_epoch())
		.count();
	return ns;
}*/

#define delay(t) do{}while(0)

#include "..\Serial.h"

/*
class ESP32
{
public:
	void restart(void) {};
};
ESP32 ESP;
*/
#endif
