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

	AHRS *ahrs;  //for te gyro

public:
	auto2();
};




#endif /* SRC_AUTONOMOUS_H_ */
