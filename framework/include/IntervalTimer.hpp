/*****************************************************************************/
/**
* Szilveszter Zsigmond 
* 2018-11-17
*
* Interval Timer Header
*
******************************************************************************/

#pragma once

#include <signal.h>
#include <time.h>
#include <stdint.h>

#include "DriverFramework.hpp"

namespace DriverFramework
{
    #define CLOCKID CLOCK_REALTIME
    #define SIG SIGRTMIN        // linux realtime signal MIN,  see the "Real-time signals" section of signal(7) man page 
    #define MAXTIMERNR 5

    typedef void (*IntervalTimerCallback)(void * arg);

    typedef struct {
        timer_t timerId;
        IntervalTimerCallback callback;
        void * arg;
        bool available = true;
    } TimerContainer;

    class IntervalTimer{
    public:
        IntervalTimer(uint64_t interval){
            _interval = interval;
        }
        void setCallback( IntervalTimerCallback cb, void *arg);
        void initTimer(void);
        static uint8_t timerCount;
        static void addTimer(timer_t intervalTimerID, IntervalTimerCallback callback, void * arg);
        static TimerContainer timers[MAXTIMERNR];

    private:
        uint64_t _interval; // interval in nanosec
        IntervalTimerCallback callback;
        void * arg;
        timer_t intervalTimerID;
        struct sigaction sa;
        struct sigevent se;
        struct itimerspec its;

        void startTimer(void);
        void stopTimer(void);
        void deleteTimer(void);
    };
}