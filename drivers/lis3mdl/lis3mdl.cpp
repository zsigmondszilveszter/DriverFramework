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

#include <string.h>
#include "DriverFramework.hpp"
#include "lis3mdl.hpp"


#define LIS3MDL_BUS_FREQUENCY_IN_KHZ	(400)
// Found through trial and error, a timeout of 100 us seems to fail.
#define LIS3MDL_TRANSFER_TIMEOUT_IN_USECS (500)


using namespace DriverFramework;


int LIS3MDL::lis3mdl_init()
{
	/* Zero the struct */
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

	int result;
	uint8_t sensor_id;

	/* Read the IDs of the LIS3MDL sensor to confirm it's presence. */
	result = _readReg(LIS3MDL_WHO_AM_I, &sensor_id, sizeof(sensor_id));
	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the lis3mdl mag sensor");
		return -EIO;
	}
	if (sensor_id != LIS3MDL_ID) {
		DF_LOG_ERR("LIS3MDL sensor ID_A returned 0x%x instead of 0x%x", sensor_id, LIS3MDL_ID);
		return -1;
	}


    result = _writeReg(LIS3MDL_CTRL_REG1, (uint8_t) 0b11100010); // TEMP_EN=1, OM0/OM1=11, ultra high performance, ODR=1
	if (result != 0) {
		DF_LOG_ERR("error: LIS3MDL sensor configuration  failed");
		return -EIO;
	}
    result = _writeReg(LIS3MDL_CTRL_REG2, (uint8_t) 0b00000000 | BITS_FS_M_4Gs); // set full scale range
    if (result != 0) {
		DF_LOG_ERR("error: LIS3MDL sensor configuration  failed");
		return -EIO;
	}
    set_mag_scale(BITS_FS_M_4Gs);

    result = _writeReg(LIS3MDL_CTRL_REG3, (uint8_t) 0b00000000); // MD[1:0] = 00 - Continuous-conversion mode
	if (result != 0) {
		DF_LOG_ERR("error: LIS3MDL sensor configuration  failed");
		return -EIO;
	}
    result = _writeReg(LIS3MDL_CTRL_REG4, (uint8_t) 0b00001100); // OMZ = 11, ultra-high performance 
	if (result != 0) {
		DF_LOG_ERR("error: LIS3MDL sensor configuration  failed");
		return -EIO;
	}

	usleep(1000);

    DF_LOG_INFO("LIS3MDL Magnetometer sensor configuration succeeded");
	return 0;
}

void LIS3MDL::set_mag_scale(int scale)
{
	switch (scale) {
	case BITS_FS_M_4Gs:
		_mag_scale = 0.00014f;
		break;

	case BITS_FS_M_8Gs:
		_mag_scale = 0.00029f;
		break;

	case BITS_FS_M_12Gs:
		_mag_scale = 0.00043f;
		break;

	case BITS_FS_M_16Gs:
		_mag_scale = 0.00058f;
		break;
	}
}

int LIS3MDL::start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start I2CDevObj");
		goto start_exit;
	}

	/* Configure the I2C bus parameters for the mag sensor. */
	result = _setSlaveConfig(LIS3MDL_SLAVE_ADDRESS,
				 LIS3MDL_BUS_FREQUENCY_IN_KHZ,
				 LIS3MDL_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto start_exit;
	}

	/* Initialize the mag sensor. */
	result = lis3mdl_init();

	if (result != 0) {
		DF_LOG_ERR("error: mag sensor initialization failed, sensor read thread not started");
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

int LIS3MDL::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

void LIS3MDL::_measure()
{
    int16_t resultdata;
    struct hmc_report report {};

    // Magnetometer
	read2_i2c_registerLSB(LIS3MDL_OUT_X_L, &resultdata);
	report.x = resultdata;
	read2_i2c_registerLSB(LIS3MDL_OUT_Y_L, &resultdata);
	report.y = resultdata;
	read2_i2c_registerLSB(LIS3MDL_OUT_Z_L, &resultdata);
	report.z = resultdata;

    m_sensor_data.field_x_ga = float(report.x) * _mag_scale;
    m_sensor_data.field_y_ga = float(report.y) * _mag_scale;
    m_sensor_data.field_z_ga = float(report.z) * _mag_scale;
    m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
    m_sensor_data.read_counter++;

    _publish(m_sensor_data);
}

/** 
 * Read 2 byte value from register pointed by r_addr 
 * and from its pair - incremented addr - 2 * 8bit register = 16 bit data (2 byte)
 */
int LIS3MDL::read2_i2c_registerLSB(uint8_t address, int16_t *resultdata){
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