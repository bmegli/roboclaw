/*
 * roboclaw-test example for roboclaw library
 * 
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

/*
 *  This example:
 *  -initializes communication with roboclaw unit 
 *  -reads & prints battery voltage
 *  -in a loop
 *    - reads duty cycle from user [-100, 100]%
 *    - sets this duty cycle to motors
 *    - breaks the loop on non numeric input
 *  -stops the motors
 *  -cleans after library
 * 
 *  Example uses fixed baudrate 38400
 * 
 *  Program expects terminal device and roboclow address (from 0x80 to 0x87, ignored in USB mode) e.g.
 * 
 *  roboclaw-test /dev/ttyACM0 0x80
 * 
 */
 
#include "../roboclaw.h"

#include <stdio.h>  //printf
#include <stdlib.h> //exit
#include <unistd.h> //sleep

void usage(char **argv);
void informative_terminate(struct roboclaw  *rc);

int main(int argc, char **argv)
{
	struct roboclaw rc;
	uint8_t address=0x80; //address of roboclaw unit
	int16_t voltage;
	float voltage_float;
	int duty_cycle;
	
	if(argc != 3)
	{
		usage(argv);
		return 0;
	}

	address = (uint8_t)strtol(argv[2], NULL, 0);
	
	//initialize at supplied terminal with default baudrate 38400
	if(roboclaw_init(&rc, argv[1], B38400) != ROBOCLAW_OK)
	{
		perror("unable to initialize roboclaw");
		exit(1);
	}

	printf("initialized communication with roboclaw\n");
	
	//read the battery voltage
	if(roboclaw_main_battery_voltage(&rc, address, &voltage) != ROBOCLAW_OK)
		informative_terminate(&rc);

	voltage_float = (float)voltage/10.0f;
	printf("battery voltage is : %f V\n", voltage_float);

	printf("WARNING - make sure it is safe for the motors to be moved\n\n");

	while(1)
	{
		printf("enter duty cycle [-100, 100] or 'q'  to quit\n");

		if( scanf("%d", &duty_cycle) < 1 )
			break;
			
		if( duty_cycle > 100 )
			duty_cycle = 100;
			
		if( duty_cycle < -100 )
			duty_cycle = -100;
			
		//32767 is max duty cycle setpoint that roboclaw accepts
		duty_cycle = (float)duty_cycle/100.0f * 32767;	
		
		if(roboclaw_duty_m1m2(&rc, address, duty_cycle, duty_cycle) != ROBOCLAW_OK )
		{
			fprintf(stderr, "problem communicating with roboclaw, terminating\n");
			break;			
		}			
	}
	
	//make sure the motors are stopped before leaving
	printf("stopping the motors..\n");
	roboclaw_duty_m1m2(&rc, address, 0, 0);
		
	if(roboclaw_close(&rc) != ROBOCLAW_OK)
		perror("unable to shutdown roboclaw cleanly");

	printf("bye...\n");

	return 0;
}

void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s terminal_device address\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0 0x80\n", argv[0]);
	printf("%s /dev/ttyAMA0 0x80\n", argv[0]);
	printf("%s /dev/tty_in1 0x80\n", argv[0]);
}

void informative_terminate(struct roboclaw *rc)
{
	fprintf(stderr, "problem communicating with roboclaw\n");
	fprintf(stderr, "make sure you are:\n");
	fprintf(stderr, "-using correct tty device\n");
	fprintf(stderr, "-using correct address\n");
	fprintf(stderr, "-set correct roboclaw baudrate (hardcoded 38400 in example)\n");
	fprintf(stderr, "-wired things correctly\n");
	fprintf(stderr, "terminating...\n");
	roboclaw_close(rc);
	exit(1);
}
