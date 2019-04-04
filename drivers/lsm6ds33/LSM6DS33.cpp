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

#define G_SI 9.80665f
#define PI 3.14159f

using namespace DriverFramework;

/**
 * 
 */
void LSM6DS33::set_gyro_scale(int scale)
{
	switch (scale) {
	case LSM6DS33_BITS_FS_G_245DPS:
		_gyro_scale = PI / 180.0f * 0.00875f;
		break;
	case LSM6DS33_BITS_FS_G_500DPS:
		_gyro_scale = PI / 180.0f * 0.0175f;
		break;
	case LSM6DS33_BITS_FS_G_1000DPS:
		_gyro_scale = PI / 180.0f * 0.035f;
		break;
	case LSM6DS33_BITS_FS_G_2000DPS:
		_gyro_scale = PI / 180.0f * 0.07f;
		break;
	}
}

/**
 * 
 */
void LSM6DS33::set_acc_scale(int scale)
{
	switch (scale) {
	case LSM6DS33_BITS_FS_XL_2G:
		_acc_scale = 0.000061f * G_SI;
		break;

	case LSM6DS33_BITS_FS_XL_4G:
		_acc_scale = 0.000122f * G_SI;
		break;

	case LSM6DS33_BITS_FS_XL_8G:
		_acc_scale = 0.000244f * G_SI;
		break;

	case LSM6DS33_BITS_FS_XL_16G:
		_acc_scale = 0.000488f * G_SI;
		break;
	}
}

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

	/* Read the ID of the LSM6DS33 sensor to confirm it's presence. */
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
	// enable accelerometer X,Y,Z axes (although they are enbled default)
	result = _writeReg( LSM6DS33_CTRL9_XL, (uint8_t)0x38 );
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Accel");
		return -EIO;
	}
	// enable gyroscope X,Y,Z axes (although they are enbled default)
	result = _writeReg( LSM6DS33_CTRL10_C,(uint8_t) 0x38 );
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Gyro");
		return -EIO;
	}
	
	
	// Accelerometer operating mode selection. Note: see CTRL1_XL (10h) register description for details
	result = _writeReg( LSM6DS33_CTRL1_XL, (uint8_t)(0x60 | LSM6DS33_BITS_FS_XL_16G | LSM6DS33_BITS_BW_XL_400HZ) ); // 0x60 = 416 Hz (High Performance)
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Accel");
		return -EIO;
	}
	set_acc_scale(LSM6DS33_BITS_FS_XL_16G);

	// Gyroscope operating mode selection. Note: see CTRL2_G (10h) register description for details
	result = _writeReg(LSM6DS33_CTRL2_G, (uint8_t)(0x60 | LSM6DS33_BITS_FS_G_2000DPS) ); //  0x60 = 416 Hz (High Performance)
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Gyro");
		return -EIO;
	}
	set_gyro_scale(LSM6DS33_BITS_FS_G_2000DPS);


	// result = _writeReg(LSM6DS33_CTRL7_G, (char) 0b10000000 ); // Gyroscope: disable high performance mode, enable normal mode
	// if (result != 0) {
	// 	DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor");
	// 	return -EIO;
	// }
	// result = _writeReg(LSM6DS33_CTRL6_C, (char) 0b00010000 ); // Accelerometer: disable high performance mode, enable normal mode
	// if (result != 0) {
	// 	DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor");
	// 	return -EIO;
	// }

	
	

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
	int result;
	// Leave the IMU in a reset state (turned off).
	result = _writeReg( LSM6DS33_CTRL1_XL,(char) 0b00000000 ); // PowerDown
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Accel");
		return -EIO;
	}
	result = _writeReg(LSM6DS33_CTRL2_G, (char) 0b00000000 ); // PowerDown
	if (result != 0) {
		DF_LOG_ERR("error: unable to configure LSM6DS33 IMU sensor Gyro");
		return -EIO;
	}

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
	lsm6ds33_deinit();

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

	// Check if the full accel range of the accel has been used. If this occurs, it is
	// either a spike due to a crash/landing or a sign that the vibrations levels
	// measured are excessive.
	if (report.accel_x == INT16_MIN || report.accel_x == INT16_MAX ||
	    report.accel_y == INT16_MIN || report.accel_y == INT16_MAX ||
	    report.accel_z == INT16_MIN || report.accel_z == INT16_MAX) {
		++m_sensor_data.accel_range_hit_counter;
	}

	// Also check the full gyro range, however, this is very unlikely to happen.
	if (report.gyro_x == INT16_MIN || report.gyro_x == INT16_MAX ||
	    report.gyro_y == INT16_MIN || report.gyro_y == INT16_MAX ||
	    report.gyro_z == INT16_MIN || report.gyro_z == INT16_MAX) {
		++m_sensor_data.gyro_range_hit_counter;
	}

	m_sensor_data.accel_m_s2_x = report.accel_x * _acc_scale;
	m_sensor_data.accel_m_s2_y = report.accel_y * _acc_scale;
	m_sensor_data.accel_m_s2_z = report.accel_z * _acc_scale;

	m_sensor_data.temp_c = float(report.temp) / 16.0f  + 25.0f;
	m_sensor_data.gyro_rad_s_x = float(report.gyro_x) * _gyro_scale;
	m_sensor_data.gyro_rad_s_y = float(report.gyro_y) * _gyro_scale;
	m_sensor_data.gyro_rad_s_z = float(report.gyro_z) * _gyro_scale;

	++m_sensor_data.read_counter;

	_publish(m_sensor_data);

	#if defined MEASURE_MEASURE_TIME && MEASURE_MEASURE_TIME == true
	Utilities::measureAndPrintRealTime(begin_time, (char *) "LSM6DS33");
	#endif 

	// DF_LOG_INFO("Temperature: %.2f", (double) m_sensor_data.temp_c);
	// DF_LOG_INFO("gyro x: %7.4f, y: %7.4f, z: %7.4f", (double)m_sensor_data.gyro_rad_s_x, (double)m_sensor_data.gyro_rad_s_y, (double)m_sensor_data.gyro_rad_s_z);
	// DF_LOG_INFO("accel x: %6.2f, y: %6.2f, z: %6.2f", (double)m_sensor_data.accel_m_s2_x, (double)m_sensor_data.accel_m_s2_y, (double)m_sensor_data.accel_m_s2_z);

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