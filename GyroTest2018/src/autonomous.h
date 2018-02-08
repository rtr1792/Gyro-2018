/*
 * autonomous.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

class auto1 {
private:
	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

public:
   auto1();
};


class auto2 {
private:
	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

	WPI_TalonSRX *intake1; //intake motor (left?)
	WPI_TalonSRX *intake2; //intake motor (right?)

	WPI_TalonSRX *lift1; //lift motor (top?)
	WPI_TalonSRX *lift2; //lift motor (bottom?)

	AHRS *ahrs;  //for te gyro

public:
	auto2();
};

class auto3 {
private:
	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

	WPI_TalonSRX *intake1; //intake motor (left?)
	WPI_TalonSRX *intake2; //intake motor (right?)

	WPI_TalonSRX *lift1; //lift motor (top?)
	WPI_TalonSRX *lift2; //lift motor (bottom?)

	AHRS *ahrs;  //for te gyro

public:
	auto3();
};

class auto4 {
private:
	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

	WPI_TalonSRX *intake1; //intake motor (left?)
	WPI_TalonSRX *intake2; //intake motor (right?)

	WPI_TalonSRX *lift1; //lift motor (top?)
	WPI_TalonSRX *lift2; //lift motor (bottom?)

	AHRS *ahrs;  //for te gyro

public:
	auto4();
};




#endif /* SRC_AUTONOMOUS_H_ */
