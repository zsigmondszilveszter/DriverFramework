/**
 * Szilveszter Zsigmond 2019-03-29 
 */

#include <time.h>
#include <sys/time.h>
#include "Utilities.hpp"
#include "DriverFramework.hpp"

using namespace DriverFramework;

/* ************************************************************************** */
// Wall time
/* ************************************************************************** */
double Utilities::getRealClock(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec * 1000000 + (double)time.tv_usec;
}

/* ************************************************************************** */
// Wall time
/* ************************************************************************** */
double Utilities::startRealTimeMeasure(){
    return getRealClock();
}

/* ************************************************************************** */
// return the elapsed time from startTime in milisec in Real time
/* ************************************************************************** */
double Utilities::measureRealTime(double startTime){
    double end = getRealClock();
    return (double)(end - startTime) / 1000;
}

/* ************************************************************************** */
// print to standard output the elapsed time from startTime in milisec in Real time
/* ************************************************************************** */
void Utilities::measureAndPrintRealTime(double startTime){
    DF_LOG_ERR("Time elapsed: %0.6f msec", measureRealTime(startTime));
}