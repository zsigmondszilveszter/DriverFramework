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
    #define MAG_DEVICE_PATH "/dev/i2c-2"

    // update frequency 250 Hz
    // #define MAG3110_MEASURE_INTERVAL_US 4000
    #define MAG3110_MEASURE_INTERVAL_US 100000

    // TODO: include some common header file (currently in drv_sensor.h).
    #define DRV_DF_DEVTYPE_MAG3110 0x67

    #define MAG3110_ID 0xC4

    #define MAG3110_SLAVE_ADDRESS 0x0E  // slave address

    enum MAG3110RegAddr
    {
        MAG3110_DR_STATUS   = 0x00,  
        MAG3110_OUT_X_MSB   = 0x01,
        MAG3110_OUT_X_LSB   = 0x02,
        MAG3110_OUT_Y_MSB   = 0x03,
        MAG3110_OUT_Y_LSB   = 0x04,
        MAG3110_OUT_Z_MSB   = 0x05,
        MAG3110_OUT_Z_LSB   = 0x06,
        MAG3110_WHO_AM_I    = 0x07,
        MAG3110_SYSMOD      = 0x08,
        MAG3110_OFF_X_MSB   = 0x09,
        MAG3110_OFF_X_LSB   = 0x0A,
        MAG3110_OFF_Y_MSB   = 0x0B,
        MAG3110_OFF_Y_LSB   = 0x0C,
        MAG3110_OFF_Z_MSB   = 0x0D,
        MAG3110_OFF_Z_LSB   = 0x0E,
        MAG3110_DIE_TEMP    = 0x0F,
        MAG3110_CTRL_REG1   = 0x10,
        MAG3110_CTRL_REG2   = 0x11,
    };

    #define AUTO_MRST_EN    0x80
    #define RAW_MODE        0x20
    #define ZYXDR           0x08

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


    class MAG3110 : public MagSensor
    {
    public:
        MAG3110(const char *device_path) :
            MagSensor(device_path, MAG3110_MEASURE_INTERVAL_US),
            _measurement_requested(false)
        {
            m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MAG3110;
            m_id.dev_id_s.address = MAG3110_SLAVE_ADDRESS;
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
        int mag3110_init();

        //struct mag3110_sensor_calibration 	m_sensor_calibration;

        // we need to request a measurement before we can collect it
        bool _measurement_requested;

        float _mag_scale;
        void set_mag_scale(int scale);

        int read2_i2c_registerMSB(uint8_t address, int16_t *resultdata);
    };

}; // namespace DriverFramework