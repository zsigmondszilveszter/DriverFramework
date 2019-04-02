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

#include <errno.h>
#include <math.h>
#include "DriverFramework.hpp"
#include "LSM6DS33.hpp"
#include "Utilities.hpp"

#define MEASURE_MEASURE_TIME false

using namespace DriverFramework;

/**
 * 
 */
int LSM6DS33::set_i2c_slave_config()
{
	int result = 0;
	result = _setSlaveConfig(LSM6DS33_SLAVE_ADDRESS, LSM6DS33_I2C_BUS_FREQUENCY_IN_KHZ, LSM6DS33_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config, result: %d", result);
		DF_LOG_ERR("ERRNO: %d", errno);
	}
	return result;
}

/**
 * 
 */
int LSM6DS33::lsm6ds33_init()
{
	/* Zero the struct */
	m_sensor_data.accel_m_s2_x = 0.0f;
	m_sensor_data.accel_m_s2_y = 0.0f;
	m_sensor_data.accel_m_s2_z = 0.0f;
	m_sensor_data.gyro_rad_s_x = 0.0f;
	m_sensor_data.gyro_rad_s_y = 0.0f;
	m_sensor_data.gyro_rad_s_z = 0.0f;
	m_sensor_data.temp_c = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	//
	set_i2c_slave_config();

	int result;
	uint8_t sensor_id;

	/* Read the ID of the BMP180 sensor to confirm it's presence. */
	result = _readReg(LSM6DS33_WHO_AM_I, &sensor_id, sizeof(sensor_id));
	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the LSM6DS33 IMU sensor");
		return -EIO;
	}
	if (sensor_id != LSM6DS33_ID) {
		DF_LOG_ERR("LSM6DS33 imu sensor ID returned 0x%x instead of 0x%x", sensor_id, LSM6DS33_ID);
		return -1;
	}

	// init
	// Accelerometer operating mode selection. Note: see CTRL1_XL (10h) register description for details
    // write_i2c_register(FD_ImuIIC, CTRL1_XL,(char) 0b01100000 ); // 416 Hz (High Performance)
	result = _writeReg( LSM6DS33_CTRL1_XL,(char) 0b01010000 ); // 208 Hz
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Gyro");
		return -EIO;
	}

	// Gyroscope operating mode selection. Note: see CTRL2_G (10h) register description for details
	// write_i2c_register(FD_ImuIIC, CTRL2_G, (char) 0b01100000 ); // 416 Hz (High Performance)
	result = _writeReg(LSM6DS33_CTRL2_G, (char) 0b01010000 ); // 208 Hz
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Accel");
		return -EIO;
	}

	// write_i2c_register(FD_ImuIIC, CTRL7_G, (char) 0b10000000); // Gyroscope: disable high performance mode, enable normal mode
	// write_i2c_register(FD_ImuIIC, CTRL6_C, (char) 0b00010000); // Accelerometer: disable high performance mode, enable normal mode

	// measure times
	#if defined MEASURE_MEASURE_TIME && MEASURE_MEASURE_TIME == true
	begin_time = Utilities::startRealTimeMeasure();
	#endif


	DF_LOG_INFO("LSM6DS33 IMU sensor configuration succeeded");
	return 0;
}

/**
 * 
 */
int LSM6DS33::lsm6ds33_deinit()
{
	// Leave the IMU in a reset state (turned off).
	// int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	// if (result != 0) {
	// 	DF_LOG_ERR("reset failed");
	// }

	return 0;
}

/**
 * 
 */
int LSM6DS33::start()
{
	return _start();
}

/**
 * 
 */
int LSM6DS33::_start()
{
	//
	int result = I2CDevObj::start();
	if (result != 0) {
		DF_LOG_ERR("error: could not start I2CDevObj");
		goto start_exit;
	}

	/* Configure the I2C bus parameters for the IMU sensor. */
	result = set_i2c_slave_config();
	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto start_exit;
	}

	/* Initialize the IMU sensor for active and continuous operation. */
	result = lsm6ds33_init();
	if (result != 0) {
		DF_LOG_ERR("error: LSM6DS33 imu sensor initialization failed, sensor read thread not started");
		goto start_exit;
	}

	//
	result = DevObj::start();
	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto start_exit;
	}

	start_exit: 
	return result;
}

/**
 * 
 */
int LSM6DS33::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

/**
 * 
 */
void LSM6DS33::_measure()
{
	_measureData();
}

/**
 * 
 */
void LSM6DS33::_measureData()
{
	#if defined MEASURE_MEASURE_TIME && MEASURE_MEASURE_TIME == true
	Utilities::measureAndPrintRealTime(begin_time, (char *) "LSM6DS33");
	#endif 

	#if defined MEASURE_MEASURE_TIME && MEASURE_MEASURE_TIME == true
	begin_time = Utilities::startRealTimeMeasure();
	#endif 

	int16_t resultdata;
	struct packet report {};

	// Accelorometer
	read2_i2c_registerLSB(LSM6DS33_OUTX_L_XL, &resultdata);
	report.accel_x = resultdata;
	read2_i2c_registerLSB(LSM6DS33_OUTY_L_XL, &resultdata);
	report.accel_y = resultdata;
	read2_i2c_registerLSB(LSM6DS33_OUTZ_L_XL, &resultdata);
	report.accel_z = resultdata;

	// Gyroscope
	read2_i2c_registerLSB(LSM6DS33_OUTX_L_G, &resultdata);
	report.gyro_x = resultdata;
	read2_i2c_registerLSB(LSM6DS33_OUTY_L_G, &resultdata);
	report.gyro_y = resultdata;
	read2_i2c_registerLSB(LSM6DS33_OUTZ_L_G, &resultdata);
	report.gyro_z = resultdata;

	// Chip temperature
	read2_i2c_registerLSB(LSM6DS33_OUT_TEMP_L, &resultdata);
	report.temp = roundf( (float) resultdata / 16 + 25 );

	m_sensor_data.accel_m_s2_x = report.accel_x;
	m_sensor_data.accel_m_s2_y = report.accel_y;
	m_sensor_data.accel_m_s2_z = -report.accel_z;

	m_sensor_data.temp_c = float(report.temp) / 16.0f  + 25.0f;
	m_sensor_data.gyro_rad_s_x = float(report.gyro_x);
	m_sensor_data.gyro_rad_s_y = float(report.gyro_y);
	m_sensor_data.gyro_rad_s_z = -float(report.gyro_z);

	++m_sensor_data.read_counter;

	_publish(m_sensor_data);

	#if defined MEASURE_MEASURE_TIME && MEASURE_MEASURE_TIME == true
	Utilities::measureAndPrintRealTime(begin_time, (char *) "LSM6DS33");
	#endif 

	// DF_LOG_DEBUG("accel x: %d, y: %d, z: %d \t gyro x: %d, y: %d, z: %d", report.accel_x, report.accel_y, report.accel_z, report.gyro_x, report.gyro_y, report.gyro_z);

}

/** 
 * Read 2 byte value from register pointed by r_addr 
 * and from its pair - incremented addr - 2 * 8bit register = 16 bit data (2 byte)
 */
int LSM6DS33::read2_i2c_registerLSB(uint8_t address, int16_t *resultdata){
	int result;
	uint8_t pdata[2];

	result = _readReg(address, pdata, 2);
	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return -1;
	}
	*resultdata = (pdata[1] << 8) + pdata[0];

	return 0;
}