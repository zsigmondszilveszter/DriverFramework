/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include "BaroSensor.hpp"

namespace DriverFramework
{

struct bmp180_sensor_calibration {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
};

#define BARO_DEVICE_PATH "/dev/i2c-1"


// update frequency is 33 Hz at ultra high resolution oversampling; oss = 3
#define BMP180_MEASURE_INTERVAL_US 33000

#define BMP180_BUS_FREQUENCY_IN_KHZ 400
#define BMP180_TRANSFER_TIMEOUT_IN_USECS 9000

#define BMP180_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 3
#define BMP180_MAX_LEN_CALIB_VALUES 22
#define BMP180_OSS 3 // Oversampling settings 0..3

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_BMP180 0x42

#define BMP180_SLAVE_ADDRESS 0b1110111       /* 7-bit slave address */


class BMP180 : public BaroSensor
{
public:
	BMP180(const char *device_path) : BaroSensor(device_path, BMP180_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_BMP180;
		m_id.dev_id_s.address = BMP180_SLAVE_ADDRESS;
	}

	// @return 0 on success, -errno on failure
	virtual int start() override;

	// @return 0 on success, -errno on failure
	virtual int stop() override;

protected:
	virtual void _measure() override;
	virtual int _publish(struct baro_sensor_data &data) = 0;

private:
	int loadCalibration();

	// returns 0 on success, -errno on failure
	int BMP180_init();

	struct bmp180_sensor_calibration 	m_sensor_calibration;

	int _start();
	void _measureData();

	int set_i2c_slave_config();
	void _measureDataRC();

	double begin_time; // store the time measurement in it, if it is desired
};

}; // namespace DriverFramework
