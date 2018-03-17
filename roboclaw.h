/*
 * roboclaw library header
 * 
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <termios.h> //termios, baudrate constants like B115200
#include <stdint.h> //uint8_t, int16_t

//internal library data
struct roboclaw;

//error codes returned by the functions
enum {ROBOCLAW_ERROR=-1, ROBOCLAW_RETRIES_EXCEEDED=-2, ROBOCLAW_OK=0};

/* Initialize roboclaw communication with default values
 * 
 * ROBOCLAW_FAILURE is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - rc - pointer to library internal data
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0)
 * - baudrate - as defined in termios.h (e.g. B38400, B115200, ... many systems also define higher baudrates than standard)
 * 
 * returns:
 * ROBOCLAW_OK on success, ROBOCLAW_FAILURE on failure and you can query errno for details about error
 *  
*/
int roboclaw_init(struct roboclaw *rc, const char *tty, speed_t baudrate);


/* Initialize roboclaw communication
 * 
 * ROBOCLAW_FAILURE is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - rc - pointer to library internal data
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0)
 * - baudrate - as defined in termios.h (e.g. B38400, B115200, ... many systems also define higher baudrates than standard)
 * - retries - the number of retries the library should make before reporting failure (both timeout and incorrect crc)
 * - timeout_ms - timeout in ms for the roundtrip sending command and waiting for ACK or response
 * - strict_0xFF_ACK - require strict 0xFF ACK byte matching (treat non 0xFF as crc failure
 * 
 * returns:
 * ROBOCLAW_OK on success, ROBOCLAW_FAILURE on failure and you can query errno for details about error
 *  
*/
int roboclaw_init_ext(struct roboclaw *rc, const char *tty, speed_t baudrate, int timeout_ms, int retries, int strict_0xFF_ACK);


/*
 * Cleans after library 
 *
 * Closes file descriptor and resets terminal to initial settings
 * 
 * parameters:
 * rc - pointer to library internal data
 * 
 * returns:
 * - ROBOCLAW_OK on success, ROBOCLAW_FAILURE on failure and errno is set
 */
int roboclaw_close(struct roboclaw *rc);


/*
 * All the commands return:
 * - ROBOCLAW_OK on success
 * - ROBOCLAW_ERROR on IO error with errno set
 * - ROBOCLAW_RETRIES_EXCEEDED if sending didn't result in reply or the checksum didn't match `replies` times (configurable)
 * 
 * The failure codes are guarateed to have negative values
 * so in simplest scenario it is enough to test for ROBOCLAW_OK
 * 
 * All the commands take parametrs:
 * - rc - pointer to library internal data
 * - address - the address to roboclaw that should receive command
 * - ...
 * - and command specific parameters
 * 
 */

int roboclaw_duty_m1m2(struct roboclaw *rc, uint8_t address, int16_t duty_m1, int16_t duty_m2);
int roboclaw_speed_m1m2(struct roboclaw *rc,uint8_t address, int speed_m1, int speed_m2);
int roboclaw_speed_accel_m1m2(struct roboclaw *rc,uint8_t address, int speed_m1, int speed_m2, int accel);
int roboclaw_main_battery_voltage(struct roboclaw *rc,uint8_t address, int16_t *voltage);
int roboclaw_encoders(struct roboclaw *rc, uint8_t address, int32_t *enc_m1, int32_t *enc_m2);


// Implementation details, this was left in header file only so
// that you can use the library without dynamic memory allocation 
// The alternative is predeclaration and malloc in init functions 

//buffer has to be big enough to accomodate both command and its reply
enum {ROBOCLAW_BUFFER_SIZE=128};

struct roboclaw
{
	int fd;
	int timeout_ms;
	int retries;
	int strict_0xFF_ACK;
	struct termios initial_termios;
	struct termios actual_termios;
	uint8_t buffer[ROBOCLAW_BUFFER_SIZE];
};

#endif // ROBOCLAW_H
