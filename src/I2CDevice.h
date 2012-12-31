/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Name        : I2CDevice.h
 * Author      : Georgi Todorov
 * Version     :
 * Created on  : Dec 30, 2012
 *
 * Copyright © 2012 Georgi Todorov  <terahz@geodar.com>
 */

#ifndef I2CDEVICE_H_
#define I2CDEVICE_H_
#include <inttypes.h>

#define BUFFER_SIZE 0x08  //1 byte buffer


class I2CDevice {
public:
	I2CDevice(int, int);
	virtual ~I2CDevice();
	uint8_t dataBuffer[BUFFER_SIZE];
	uint8_t read_byte(int, uint8_t);
	void write_byte(int, uint8_t, uint8_t);
	int openfd();
private:
	int _i2caddr;
	int _i2cbus;
	char busfile[64];
};

#endif /* I2CDEVICE_H_ */
