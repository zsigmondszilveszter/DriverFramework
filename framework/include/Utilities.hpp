/**
 * Szilveszter Zsigmond 2019-03-29 
 */

#pragma once

#include "DriverFramework.hpp"

namespace DriverFramework
{
    class Utilities 
    { 
    public:
        static double startRealTimeMeasure();
        static void measureAndPrintRealTime(double startTime);

    protected:
        static double getRealClock();
        static double measureRealTime(double startTime);
    };
};