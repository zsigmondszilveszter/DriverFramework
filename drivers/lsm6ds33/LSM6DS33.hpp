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

#define __IMU_USE_I2C

#include "ImuSensor.hpp"

namespace DriverFramework
{
    /************************** LSM6DS33 Definitions *****************************/
    #define LSM6DS33_SLAVE_ADDRESS  0b1101011  // 0b1101011 - the IMU initial default slave addr, SA0 default state

    #define LSM6DS33_FUNC_CFG_ACCESS         0x01
    #define LSM6DS33_FIFO_CTRL1              0x06
    #define LSM6DS33_FIFO_CTRL2              0x07
    #define LSM6DS33_FIFO_CTRL3              0x08
    #define LSM6DS33_FIFO_CTRL4              0x09
    #define LSM6DS33_FIFO_CTRL5              0x0A
    #define LSM6DS33_ORIENT_CFG_G            0x0B
    #define LSM6DS33_INT1_CTRL               0x0D
    #define LSM6DS33_INT2_CTRL               0x0E
    #define LSM6DS33_WHO_AM_I                0X0F
    #define LSM6DS33_CTRL1_XL                0x10
    #define LSM6DS33_CTRL2_G                 0x11
    #define LSM6DS33_CTRL3_C                 0x12
    #define LSM6DS33_CTRL4_C                 0x13
    #define LSM6DS33_CTRL5_C                 0x14
    #define LSM6DS33_CTRL6_C                 0x15
    #define LSM6DS33_CTRL7_G                 0x16
    #define LSM6DS33_CTRL8_XL                0x17
    #define LSM6DS33_CTRL9_XL                0x18
    #define LSM6DS33_CTRL10_C                0x19
    #define LSM6DS33_WAKE_UP_SRC             0x1B
    #define LSM6DS33_TAP_SRC                 0x1C
    #define LSM6DS33_D6D_SRC                 0x1D
    #define LSM6DS33_STATUS_REG              0x1E
    #define LSM6DS33_OUT_TEMP_L              0x20
    #define LSM6DS33_OUT_TEMP_H              0x21
    #define LSM6DS33_OUTX_L_G                0x22
    #define LSM6DS33_OUTX_H_G                0x23
    #define LSM6DS33_OUTY_L_G                0x24
    #define LSM6DS33_OUTY_H_G                0x25
    #define LSM6DS33_OUTZ_L_G                0x26
    #define LSM6DS33_OUTZ_H_G                0x27
    #define LSM6DS33_OUTX_L_XL               0x28
    #define LSM6DS33_OUTX_H_XL               0x29
    #define LSM6DS33_OUTY_L_XL               0x2A
    #define LSM6DS33_OUTY_H_XL               0x2B
    #define LSM6DS33_OUTZ_L_XL               0x2C
    #define LSM6DS33_OUTZ_H_XL               0x2D
    #define LSM6DS33_FIFO_STATUS1            0x3A
    #define LSM6DS33_FIFO_STATUS2            0x3B  
    #define LSM6DS33_FIFO_STATUS3            0x3C
    #define LSM6DS33_FIFO_STATUS4            0x3D
    #define LSM6DS33_FIFO_DATA_OUT_L         0x3E
    #define LSM6DS33_FIFO_DATA_OUT_H         0x3F
    #define LSM6DS33_TIMESTAMP0_REG          0x40
    #define LSM6DS33_TIMESTAMP1_REG          0x41
    #define LSM6DS33_TIMESTAMP2_REG          0x42
    #define LSM6DS33_STEP_TIMESTAMP_L        0x49
    #define LSM6DS33_STEP_TIMESTAMP_H        0x4A
    #define LSM6DS33_STEP_COUNTER_L          0x4B
    #define LSM6DS33_STEP_COUNTER_H          0x4C
    #define LSM6DS33_FUNC_SR                 0x53
    #define LSM6DS33_TAP_CFG                 0x58
    #define LSM6DS33_TAP_THS_6D              0x59
    #define LSM6DS33_INT_DUR2                0x5A
    #define LSM6DS33_WAKE_UP_THS             0x5B
    #define LSM6DS33_WAKE_UP_DUR             0x5C
    #define LSM6DS33_FREE_FALL               0x5D
    #define LSM6DS33_MD1_CFG                 0x5E
    #define LSM6DS33_MD2_CFG                 0x5F

    // update frequency 350 Hz
    // #define LSM6DS33_MEASURE_INTERVAL_US 2857
    // update frequency 250 Hz
    #define LSM6DS33_MEASURE_INTERVAL_US 4000
    //
    #define DRV_DF_DEVTYPE_LSM6DS33 0x65

    #define LSM6DS33_ID 0x69

    #define LSM6DS33_I2C_BUS_FREQUENCY_IN_KHZ 400
    #define LSM6DS33_TRANSFER_TIMEOUT_IN_USECS 900


    #pragma pack(push, 1)
    struct packet {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };
    #pragma pack(pop)


    class LSM6DS33: public ImuSensor
    {
    public:
        LSM6DS33(const char *acc_gyro_device_path, const char *mag_device_path, bool mag_enabled = false) :
		ImuSensor(acc_gyro_device_path, LSM6DS33_MEASURE_INTERVAL_US /*0*/, false) // 0 instead of LSM6DS33_MEASURE_INTERVAL_US disables build in scheduling, because I use my own custom scheduler
        {
            m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_LSM6DS33;
		    m_id.dev_id_s.address = LSM6DS33_SLAVE_ADDRESS;
        }

        // @return 0 on success, -errno on failure
        int writeReg(int reg, uint8_t val)
        {
            return _writeReg(reg, val);
        }

        int readReg(uint8_t address, uint8_t &val)
        {
            return _readReg(address, val);
        }

        // @return 0 on success, -errno on failure
        virtual int start();

        // @return 0 on success, -errno on failure
        virtual int stop();

    protected:
        virtual void _measure();
        virtual int _publish(struct imu_sensor_data &data) = 0;

    private:

        // @returns 0 on success, -errno on failure
        int lsm6ds33_init();

        // @returns 0 on success, -errno on failure
        int lsm6ds33_deinit();

        int set_i2c_slave_config();

        int _start();
        void _measureData();
        int read2_i2c_registerLSB(uint8_t address, int16_t *resultdata);

        double begin_time; // store the time measurement in it, if it is desired
    };
}