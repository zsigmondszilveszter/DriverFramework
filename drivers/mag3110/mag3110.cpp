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
#include "mag3110.hpp"


#define MAG3110_BUS_FREQUENCY_IN_KHZ	(400)
// Found through trial and error, a timeout of 100 us seems to fail.
#define MAG3110_TRANSFER_TIMEOUT_IN_USECS (500)


using namespace DriverFramework;


int MAG3110::mag3110_init()
{
	/* Zero the struct */
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

	int result;
	uint8_t sensor_id;

	/* Read the IDs of the MAG3110 sensor to confirm it's presence. */
	result = _readReg(MAG3110_WHO_AM_I, &sensor_id, sizeof(sensor_id));
	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the mag3110 mag sensor");
		return -EIO;
	}
	if (sensor_id != MAG3110_ID) {
		DF_LOG_ERR("MAG3110 sensor ID_A returned 0x%x instead of 0x%x", sensor_id, MAG3110_ID);
		return -1;
	}

	// init
	result = _writeReg(MAG3110_CTRL_REG2, (uint8_t) AUTO_MRST_EN | RAW_MODE); // Enable automatic resets by setting bit AUTO_MRST_EN in CTRL_REG2. (CTRL_REG2 = 0x80)
	if (result != 0) {
		DF_LOG_ERR("error: MAG3110 sensor configuration  failed");
		return -EIO;
	}

	result = _writeReg(MAG3110_CTRL_REG1, (uint8_t) 0x01); // Put MAG3110 in active mode 80 Hz with OSR = 1 by writing 0x01 to CTRL_REG1 (CTRL_REG1 = 0x01)
	if (result != 0) {
		DF_LOG_ERR("error: MAG3110 sensor configuration  failed");
		return -EIO;
	}

	usleep(1000);

    DF_LOG_INFO("MAG3110 Magnetometer sensor configuration succeeded");
	return 0;
}

void MAG3110::set_mag_scale(int scale)
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

int MAG3110::start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start I2CDevObj");
		goto start_exit;
	}

	/* Configure the I2C bus parameters for the mag sensor. */
	result = _setSlaveConfig(MAG3110_SLAVE_ADDRESS,
				 MAG3110_BUS_FREQUENCY_IN_KHZ,
				 MAG3110_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto start_exit;
	}

	/* Initialize the mag sensor. */
	result = mag3110_init();

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

int MAG3110::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

void MAG3110::_measure()
{
    int16_t resultdata;
    struct hmc_report report {};

	// check if there is new measurement available
	uint8_t status_red;
	_readReg(MAG3110_DR_STATUS, status_red);
	if( (status_red & ZYXDR) != ZYXDR){
		// no new data
		return;
	}

    // Magnetometer
	read2_i2c_registerMSB(MAG3110_OUT_X_MSB, &resultdata);
	report.x = resultdata;
	read2_i2c_registerMSB(MAG3110_OUT_Y_MSB, &resultdata);
	report.y = resultdata;
	read2_i2c_registerMSB(MAG3110_OUT_Z_MSB, &resultdata);
	report.z = resultdata;

	// DF_LOG_INFO("MAG3110 x:%d \t y:%d \t z:%d", report.x, report.y, report.z);

	m_sensor_data.field_x_ga = double(report.x) * 0.1 * 0.01;
    m_sensor_data.field_y_ga = double(report.y) * 0.1 * 0.01;
    m_sensor_data.field_z_ga = double(report.z) * 0.1 * 0.01;
    m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
    m_sensor_data.read_counter++;

	// DF_LOG_INFO("MAG3110 x:%.4f \t y:%.4f \t z:%.4f", (double)m_sensor_data.field_x_ga * 100, (double)m_sensor_data.field_y_ga * 100, (double)m_sensor_data.field_z_ga * 100);

    _publish(m_sensor_data);
}

/** 
 * Read 2 byte value from register pointed by r_addr 
 * and from its pair - incremented addr - 2 * 8bit register = 16 bit data (2 byte)
 */
int MAG3110::read2_i2c_registerMSB(uint8_t address, int16_t *resultdata){
	int result;
	uint8_t pdata[2];

	result = _readReg(address, pdata, 2);
	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return -1;
	}
	*resultdata = (pdata[0] << 8) + pdata[1];

	return 0;
}