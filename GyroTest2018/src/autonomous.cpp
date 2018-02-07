/*
 * autonomous.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include "autonomous.h"

auto1::auto1() {
	srx1 = new WPI_TalonSRX(1);
	srx2 = new WPI_TalonSRX(2);  //left
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right
	srx6 = new WPI_TalonSRX(6);
//if(srx1->GetSensorCollection().GetQuadraturePosition()){
	srx1->Set(.5);
	srx2->Set(.5);  //left side set to 50% speed
	srx3->Set(.5);

	srx4->Set(.5);
	srx5->Set(.5);  //right side set to 50% speed
	srx6->Set(.5);
//}
//	else{
	srx1->Set(0);
	srx2->Set(0);  //left side stop
	srx3->Set(0);

	srx4->Set(0);
	srx5->Set(0);  //right side stop
	srx6->Set(0);
//	}
}

auto2::auto2() {
	srx1 = new WPI_TalonSRX(1);
	srx2 = new WPI_TalonSRX(2);  //left
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right
	srx6 = new WPI_TalonSRX(6);

	ahrs = new AHRS(SPI::Port::kMXP); //also for te gyro
	ahrs->Reset();
//if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);
//}
//	else{
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
//	}
if(ahrs->GetAngle()<89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx2->Set(.5);	//left side stop
		srx3->Set(.5);

		srx4->Set(-.5);
		srx5->Set(-.5);	//right side stop
		srx6->Set(-.5);
}
	else if(89.5 <= ahrs->GetAngle() <= 90.5 and srx1->GetSensorCollection().GetQuadraturePosition())	{
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
}




