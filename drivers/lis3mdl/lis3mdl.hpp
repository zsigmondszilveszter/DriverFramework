/****************************************************************************
 *
 *   Copyright (C) 2019 Szilveszter Zsigmond. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "MagSensor.hpp"

namespace DriverFramework
{
    #define MAG_DEVICE_PATH "/dev/i2c-0"

    // update frequency 250 Hz
    #define LIS3MDL_MEASURE_INTERVAL_US 4000

    // TODO: include some common header file (currently in drv_sensor.h).
    #define DRV_DF_DEVTYPE_LIS3MDL 0x66

    #define LIS3MDL_ID 0x3D

    #define LIS3MDL_SLAVE_ADDRESS 0b0011110  // 0b0011110 - the magnetometer initial default slave addr, SA0 default state
    enum LIS3MDLRegAddr
    {
        LIS3MDL_WHO_AM_I    = 0x0F,

        LIS3MDL_CTRL_REG1   = 0x20,
        LIS3MDL_CTRL_REG2   = 0x21,
        LIS3MDL_CTRL_REG3   = 0x22,
        LIS3MDL_CTRL_REG4   = 0x23,
        LIS3MDL_CTRL_REG5   = 0x24,

        LIS3MDL_STATUS_REG  = 0x27,
        LIS3MDL_OUT_X_L     = 0x28,
        LIS3MDL_OUT_X_H     = 0x29,
        LIS3MDL_OUT_Y_L     = 0x2A,
        LIS3MDL_OUT_Y_H     = 0x2B,
        LIS3MDL_OUT_Z_L     = 0x2C,
        LIS3MDL_OUT_Z_H     = 0x2D,
        LIS3MDL_TEMP_OUT_L  = 0x2E,
        LIS3MDL_TEMP_OUT_H  = 0x2F,
        LIS3MDL_INT_CFG     = 0x30,
        LIS3MDL_INT_SRC     = 0x31,
        LIS3MDL_INT_THS_L   = 0x32,
        LIS3MDL_INT_THS_H   = 0x33,
    };

    #define BITS_FS_M_4Gs               0x00
    #define BITS_FS_M_8Gs               0x20
    #define BITS_FS_M_12Gs              0x40
    #define BITS_FS_M_16Gs              0x60

    #pragma pack(push, 1)
    struct hmc_report{ /* status register and data as read back from the device */
        int16_t		x;
        int16_t		z;
        int16_t		y;
    };
    #pragma pack(pop)


    class LIS3MDL : public MagSensor
    {
    public:
        LIS3MDL(const char *device_path) :
            MagSensor(device_path, LIS3MDL_MEASURE_INTERVAL_US),
            _measurement_requested(false)
        {
            m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_LIS3MDL;
            m_id.dev_id_s.address = LIS3MDL_SLAVE_ADDRESS;
        }

        // @return 0 on success, -errno on failure
        virtual int start();

        // @return 0 on success, -errno on failure
        virtual int stop();

    protected:
        virtual void _measure();
        virtual int _publish(struct mag_sensor_data &data) = 0;

    private:
        int loadCalibration();

        // returns 0 on success, -errno on failure
        int lis3mdl_init();

        //struct lis3mdl_sensor_calibration 	m_sensor_calibration;

        // we need to request a measurement before we can collect it
        bool _measurement_requested;

        float _mag_scale;
        void set_mag_scale(int scale);

        int read2_i2c_registerLSB(uint8_t address, int16_t *resultdata);
    };

}; // namespace DriverFramework