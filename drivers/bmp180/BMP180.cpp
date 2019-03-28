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

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "DriverFramework.hpp"
#include "BMP180.hpp"
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#endif

#define BMP180_REG_ID 0xD0
#define BMP180_REG_CTRL_MEAS 0xF4
#define BMP180_REG_OUT_MSB 0xF6
#define BMP180_REG_PRESS_MSB 0xF7
#define BMP180_REG_CALIB_COEF 0xAA

#define BMP180_ID 0x55

#define BMP180_BITS_CTRL_MEAS_OVERSAMPLING_TEMP2X     0b01000000
#define BMP180_BITS_CTRL_MEAS_OVERSAMPLING_PRESSURE8X 0b11000000
#define BMP180_BITS_CTRL_MEAS_POWER_MODE_NORMAL	      0b00000011

#define BMP180_BITS_CONFIG_STANDBY_0MS5	0b00000000
#define BMP180_BITS_CONFIG_FILTER_OFF	0b00000000
#define BMP180_BITS_CONFIG_SPI_OFF	0b00000000

using namespace DriverFramework;

int BMP180::set_i2c_slave_config()
{
#if defined(__BARO_USE_SPI)
	return -1;
#else
#ifdef __DF_BBBLUE
	int result = rc_i2c_set_device_address(m_bus_num, BMP180_SLAVE_ADDRESS);
#else
	int result = _setSlaveConfig(BMP180_SLAVE_ADDRESS,
				     BMP180_BUS_FREQUENCY_IN_KHZ,
				     BMP180_TRANSFER_TIMEOUT_IN_USECS);
#endif


	if (result < 0) {
		DF_LOG_ERR("Could not set slave config, result: %d", result);
	}

	return result;
#endif
}

int BMP180::loadCalibration()
{
	int result;
	uint8_t calib_values[BMP180_MAX_LEN_CALIB_VALUES];
	memset(calib_values, 0, BMP180_MAX_LEN_CALIB_VALUES);

	result = _readReg(BMP180_REG_CALIB_COEF, calib_values, BMP180_MAX_LEN_CALIB_VALUES);

	if (result != 0) {
		DF_LOG_ERR("error: unable to read sensor calibration values from the sensor");
		return -EIO;
	}

	m_sensor_calibration.AC1	= (calib_values[0] 	<< 8) + calib_values[1];
	m_sensor_calibration.AC2	= (calib_values[2] 	<< 8) + calib_values[3];
	m_sensor_calibration.AC3	= (calib_values[4] 	<< 8) + calib_values[5];
	m_sensor_calibration.AC4	= (calib_values[6] 	<< 8) + calib_values[7];
	m_sensor_calibration.AC5	= (calib_values[8] 	<< 8) + calib_values[9];
	m_sensor_calibration.AC6	= (calib_values[10] << 8) + calib_values[11];
	m_sensor_calibration.B1		= (calib_values[12] << 8) + calib_values[13];
	m_sensor_calibration.B2		= (calib_values[14] << 8) + calib_values[15];
	m_sensor_calibration.MB		= (calib_values[16] << 8) + calib_values[17];
	m_sensor_calibration.MC		= (calib_values[18] << 8) + calib_values[19];
	m_sensor_calibration.MD		= (calib_values[20] << 8) + calib_values[21];

	return 0;
}

int BMP180::BMP180_init()
{
	/* Zero the struct */
	m_sensor_data.pressure_pa = 0.0f;
	m_sensor_data.temperature_c = 0.0f;
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;


	int result;
	uint8_t sensor_id;

	/* Read the ID of the BMP180 sensor to confirm it's presence. */
	result = _readReg(BMP180_REG_ID, &sensor_id, sizeof(sensor_id));

	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the BMP180 pressure sensor");
		return -EIO;
	}

	if (sensor_id != BMP180_ID) {
		DF_LOG_ERR("BMP180 sensor ID returned 0x%x instead of 0x%x", sensor_id, BMP180_ID);
		return -1;
	}

	/* Load and display the internal calibration values. */
	result = loadCalibration();

	if (result != 0) {
		DF_LOG_ERR("error: unable to complete initialization of the BMP180 pressure sensor");
		return -EIO;
	}

	DF_LOG_INFO("BMP180 sensor configuration succeeded");

	usleep(1000);
	return 0;
}

int BMP180::start()
{
	return _start();
}

int BMP180::_start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto start_exit;
	}

	/* Configure the I2C bus parameters for the pressure sensor. */

	result = set_i2c_slave_config();

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto start_exit;
	}

	/* Initialize the pressure sensor for active and continuous operation. */
	result = BMP180_init();

	if (result != 0) {
		DF_LOG_ERR("error: pressure sensor initialization failed, sensor read thread not started");
		goto start_exit;
	}


	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto start_exit;
	}

start_exit:
	return result;
}

int BMP180::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

void BMP180::_measure()
{
	_measureData();
}

void BMP180::_measureData()
{
	// begin_time = startRealTimeMeasure();

	uint8_t pdata[BMP180_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES];
	uint32_t UT, UP;
	uint8_t buffer_out[2];
	int result;

	// TEMPERATURE
	// start measurement
	buffer_out[0] = 0x2e;
	result = _writeReg(BMP180_REG_CTRL_MEAS, buffer_out, 1);
	if (result < 0) {
		DF_LOG_ERR("error: writing I2C bus failed");
		return;
	}
	// wait for measurement
	usleep(4500); 
	// read the temperature
	result = _readReg(BMP180_REG_OUT_MSB, pdata, 2);
	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return;
	}
	UT = ( pdata[0] << 8 ) + pdata[1];


	// PRESSURE
	buffer_out[0] = 0x34 + (BMP180_OSS << 6);
	result = _writeReg(BMP180_REG_CTRL_MEAS, buffer_out, 1);
	if (result < 0) {
		DF_LOG_ERR("error: writing I2C bus failed");
		return;
	}
	switch(BMP180_OSS){
		case 0: usleep(4500); break;
		case 1: usleep(7500); break;
		case 2: usleep(13500); break;
		case 3: usleep(25500); break;
		default: usleep(4500); break;
	}
	result = _readReg(BMP180_REG_OUT_MSB, pdata, 3);
	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return;
	}
	UP = (( pdata[0] << 16 ) + ( pdata[1] << 8) + pdata[2] ) >> (8 - BMP180_OSS);


	int32_t T, P;
	int32_t b3, b5, b6, x1, x2, x3;
	uint32_t b4, b7;

	// calculate temperature
	x1 = ( UT - m_sensor_calibration.AC6 ) * m_sensor_calibration.AC5 / 32768;
	x2 = m_sensor_calibration.MC * 2048 / ( x1 + m_sensor_calibration.MD );
	b5 = x1 + x2;
	T = ( b5 + 8 ) / 16;

	// calculate pressure
	b6 = b5 - 4000;
	x1 = ( m_sensor_calibration.B2 * ( b6 * b6 / 4096 )) / 2048;
	x2 = m_sensor_calibration.AC2 * b6 / 2048;
	x3 = x1 + x2;
	b3 = (( (int32_t)( m_sensor_calibration.AC1 * 4 + x3 ) << BMP180_OSS ) + 2 ) / 4;
	x1 = m_sensor_calibration.AC3 * b6 / 8192;
	x2 = ( m_sensor_calibration.B1 * ( b6 * b6 / 4096 )) / 65536;
	x3 = (( x1 + x2 ) + 2) / 4;
	b4 = m_sensor_calibration.AC4 * (uint32_t)( x3 + 32768 ) / 32768;
	b7 = ((uint32_t) UP - b3) * ( 50000 >> BMP180_OSS );
	if( b7 < 0x80000000 ){
		P = (b7 * 2) / b4;
	} else {
		P = (b7 / b4) * 2;
	}
	x1 = ( P / 256 ) * ( P / 256 );
	x1 = ( x1 * 3038 ) / 65536;
	x2 = ( -7357 * P ) / 65536;
	P = P + ( x1 + x2 + 3791 ) / 16;

	m_sensor_data.temperature_c = T * (float)0.1;
	m_sensor_data.pressure_pa = (float)P;
	m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
	m_sensor_data.read_counter++;

	// measureAndPrintRealTime(begin_time);

	_publish(m_sensor_data);
}


/* ************************************************************************** */
// Wall time
/* ************************************************************************** */
double BMP180::getRealClock(){
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
double BMP180::startRealTimeMeasure(){
    return getRealClock();
}

/* ************************************************************************** */
// return the elapsed time from startTime in milisec in Real time
/* ************************************************************************** */
double BMP180::measureRealTime(double startTime){
    double end = getRealClock();
    return (double)(end - startTime) / 1000;
}

/* ************************************************************************** */
// print to standard output the elapsed time from startTime in milisec in Real time
/* ************************************************************************** */
void BMP180::measureAndPrintRealTime(double startTime){
    DF_LOG_ERR("Time elapsed: %0.6f msec", measureRealTime(startTime));
}