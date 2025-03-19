#include "Arduino.h"

EspClass ESP;

void EspClass::restart(void)
{
}

void EspClass::timerSleep(double seconds) {
    using namespace std::chrono;

    static HANDLE timer = CreateWaitableTimer(NULL, FALSE, NULL);
    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds - estimate > 1e-7) {
        double toWait = seconds - estimate;
        LARGE_INTEGER due;
        due.QuadPart = -int64_t(toWait * 1e7);
        steady_clock::time_point  start = high_resolution_clock::now();
        SetWaitableTimerEx(timer, &due, 0, NULL, NULL, NULL, 0);
        WaitForSingleObject(timer, INFINITE);
        steady_clock::time_point end = high_resolution_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double error = observed - toWait;
        double delta = error - mean;
        mean += delta / count;
        m2 += delta * (error - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
}

std::chrono::steady_clock::time_point _start;
void EspClass::startTime()
{
    _start = std::chrono::high_resolution_clock::now();
}
unsigned long EspClass::getTime()
{
    std::chrono::steady_clock::time_point m = std::chrono::high_resolution_clock::now();
    return (m - _start).count() / 1e3;
}
