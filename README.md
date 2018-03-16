# roboclaw

This is a small C library for communication with [Roboclaw](http://www.ionmc.com/) motor controllers

## Platforms 

Library works on Unix platforms (e.g. Linux) including desktop computers and SBCs like RaspberryPi, BeagleBone or LegoMindstorms EV3 (with [ev3dev](http://www.ev3dev.org/) OS).

## Hardware

You need to connect Roboclaw (or multiple Roboclaws) via USB or uart

## Why another library

After shallow code inspection of available C/C++ libraries I made decission to write my own correctly.

| Library                                  | What I would do differently                                                |Why did they do it this way|
----------------------------------------------------------------------------------------------------------------------------------------------------|
|https://github.com/SorcererX/RoboClaw     | byte-by-byte IO, busy waiting                                              |arduino library port       |
|https://github.com/ColinHeffernan/roboclaw| byte-by-byte IO, ROS serial dependence, hardcoded 100 ms timeout           |arduino library port       |
|https://bitbucket.org/vo/libroboclaw/     | limited baudrate, command ACK not read, ACK-purge race, garbage on timeout |old firmare didn't have ACK|

byte-by-byte IO - unnecessary system calls for each byte read/written costly by itself and may result in unnecessary context changes when process is put to sleep on IO

busy waiting - waiting for available bytes on read spinning CPU at 100% until IO or timeout

ACK-purge race - Roboclaw ACK byte not read, on commands tty buffers flushed resulting in race when command is followed by read command (ACK not read, buffers flushed but after or before receiving ACK?)

garbage on timeout - when read times out library returns garbage values and there is no way to check if it was timeout

### This library design

Goals:
- ligthweight for CPU
- low level handled in the library for user
- easy for setup

Achieved by:
- block IO (not byte-by-byte), waked up by OS only when real work is waiting or timeout
- configurable auto-retry on low-level
- no dependencies, permissive license

## Scope & State

The library implements only a subset of commands that I use and I have no plans to implement all possible Roboclaw commands.

Currently implemented:

-roboclaw_duty_m1m2
-roboclaw_speed_m1m2
-roboclaw_speed_accel_m1m2
-roboclaw_main_battery_voltage
-roboclaw_encoders

## Building Instructions

### Compilers and make

``` bash
$ sudo apt-get update
$ sudo apt-get install build-essential 
```

### Getting git

``` bash
sudo apt-get install git
```

### Cloning the repository

``` bash
git clone https://github.com/bmegli/roboclaw.git
```

### Building the examples

``` bash
cd roboclaw
make all
```

## Testing

Run as sudo or add your user to dialout group:

```bash
usermod -a -G dialout your_user
```

Run `roboclaw-test` with your device
(e.g. `/dev/ttyACM0` for USB, `/dev/ttyAMA0` for RaspberryPi UART, `/dev/tty_in1` for Lego Mindstorms EV3)

``` bash
./roboclaw-test /dev/ttyACM0
```

## Using

See examples directory for more complete and commented examples with error handling.

``` C
	struct roboclaw rc;
	int16_t voltage;

	roboclaw_init(&rc, tty, B460800);

	roboclaw_main_battery_voltage(rc, 0x80, &voltage)	

	roboclaw_speed_m1m2(rc, 0x80, 1000, 1000);

	sleep(1)

	roboclaw_speed_m1m2(rc, 0x80, 0, 0);

	roboclaw_close(&rc);
```

Note - configure motors and encoders correctly in IONStudio before running speed commands


### Compiling your code

C
``` bash
gcc roboclaw.c your_program.c -o your-program
```

C++
``` bash
gcc -c roboclaw.c
g++ -c your_program.cpp
g++ roboclaw.o your_program.o -o your-program
```

## License

Library is licensed under Mozilla Public License, v. 2.0.

This is similiar to LGPL but more permissive:
- you can use it as LGPL in prioprietrary software
- unlike LGPL you may copile it statically with your code

Like in LGPL, if you modify this library, you have to make your changes publicly available.
Making a github fork of the library with your changes satisfies those requirements perfectly. 




