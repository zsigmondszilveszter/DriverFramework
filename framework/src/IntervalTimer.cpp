/*****************************************************************************/
/**
* Szilveszter Zsigmond 
* 2018-11-17
*
* Interval Timer Source File
*
******************************************************************************/


/***************************** Include Files *********************************/
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#include "DriverFramework.hpp"
#include "IntervalTimer.hpp"


using namespace DriverFramework;


/* ************************************************************************** */
// Handle the signal sent by system at Timer expiration 
/* ************************************************************************** */
void timerExpiredHandler(int sig, siginfo_t *si, void *uc){
    timer_t *tidp;
    tidp = (timer_t*) si->si_value.sival_ptr;

    uint8_t i;
    for(i=0; i<IntervalTimer::timerCount; i++){
        if(IntervalTimer::timers[i].timerId == *tidp){
            //
            void *tmpptr = IntervalTimer::timers[i].arg;
            IntervalTimerCallback cb = IntervalTimer::timers[i].callback;
            cb(tmpptr);
            break;
        }
    }
}

uint8_t IntervalTimer::timerCount = 0;
TimerContainer IntervalTimer::timers[5];

/* ************************************************************************** */
//
/* ************************************************************************** */
void IntervalTimer::addTimer(timer_t intervalTimerID, IntervalTimerCallback callback, void * arg){
    uint8_t i;
    for(i=0; i<IntervalTimer::timerCount; i++){
        if(IntervalTimer::timers[i].available){
            IntervalTimer::timers[i].available = false;
            IntervalTimer::timers[i].timerId = intervalTimerID;
            IntervalTimer::timers[i].callback = callback;
            IntervalTimer::timers[i].arg = arg;
            return;
        }
    }
    if(IntervalTimer::timerCount >= MAXTIMERNR){
        return;
    } else {
        IntervalTimer::timers[i].available = false;
        IntervalTimer::timers[i].timerId = intervalTimerID;
        IntervalTimer::timers[i].callback = callback;
        IntervalTimer::timers[i].arg = arg;
        IntervalTimer::timerCount++;
    }
}

/* ************************************************************************** */
//
/* ************************************************************************** */
void IntervalTimer::setCallback( WorkCallback cb, void * _arg){
    callback = cb;
    arg = _arg;
}

/* ************************************************************************** */
//
/* ************************************************************************** */
void IntervalTimer::initTimer(){
    /* Establish handler for timer signal */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timerExpiredHandler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIG, &sa, NULL) == -1){
        // return value is not 0, print the error and exit
        DF_LOG_ERR("sigaction error, errno=%d", errno);
        exit(errno);
    }

    /* Create the timer */
    se.sigev_notify = SIGEV_SIGNAL;
    se.sigev_signo = SIG;
    se.sigev_value.sival_ptr = &intervalTimerID;
    if ( timer_create(CLOCKID, &se, &intervalTimerID) ){
        // return value is not 0, print the error and exit
        DF_LOG_ERR("timer_create error, errno=%d", errno);
        exit(errno);
    }
    IntervalTimer::addTimer(intervalTimerID, callback, arg);

    DF_LOG_INFO("Interval Timer created, timerid: %u\n", intervalTimerID);

    startTimer();
}


/* ************************************************************************** */
// Start the timer
/* ************************************************************************** */
void IntervalTimer::startTimer(){
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = _interval;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;
    if (timer_settime(intervalTimerID, 0, &its, NULL) == -1){
        // return value is not 0, print the error and exit
        DF_LOG_ERR("timer_settime(start) error, errno=%d", errno);
        exit(errno);
    }
}


/* ************************************************************************** */
//
/* ************************************************************************** */
void IntervalTimer::stopTimer(){
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 0;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    if (timer_settime(intervalTimerID, 0, &its, NULL) == -1){
        // return value is not 0, print the error and exit
        DF_LOG_ERR("timer_settime(stop) error, errno=%d", errno);;
        exit(errno);
    }
}


/* ************************************************************************** */
//
/* ************************************************************************** */
void IntervalTimer::deleteTimer(){
    timer_delete(intervalTimerID);
}