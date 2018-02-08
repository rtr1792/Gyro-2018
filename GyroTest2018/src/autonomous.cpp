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
	srx2 = new WPI_TalonSRX(2);  //left drive
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right drive
	srx6 = new WPI_TalonSRX(6);

	intake1 = new WPI_TalonSRX(7); //intake
	intake2 = new WPI_TalonSRX(8);

	lift1 = new WPI_TalonSRX(9); //lift
	lift2 = new WPI_TalonSRX(10);

	ahrs = new AHRS(SPI::Port::kMXP); //also for te gyro
	ahrs->Reset();
if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		lift1->Set(.1);
		lift2->Set(.1);
}
	else {
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
if(ahrs->GetAngle()<89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);	//left side 50%
		srx3->Set(.5);

		srx4->Set(-.5);
		srx5->Set(-.5);	//right side 50%
		srx6->Set(-.5);
}
	else if(89.5 <= ahrs->GetAngle() and ahrs->GetAngle() <= 90.5 and srx1->GetSensorCollection().GetQuadraturePosition())	{
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
	else if (ahrs->GetAngle() > 90.5 and srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(-.2);
		srx2->Set(-.2);	//left side turn left 20% speed
		srx3->Set(-.2);

		srx4->Set(.2);
		srx5->Set(.2);	//right side turn left 20% speed
		srx6->Set(.2);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		intake1->Set(-.5);
		intake2->Set(-.5);
	}
}

auto3::auto3() {
	srx1 = new WPI_TalonSRX(1);
	srx2 = new WPI_TalonSRX(2);  //left drive
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right drive
	srx6 = new WPI_TalonSRX(6);

	intake1 = new WPI_TalonSRX(7); //intake
	intake2 = new WPI_TalonSRX(8);

	lift1 = new WPI_TalonSRX(9); //lift
	lift2 = new WPI_TalonSRX(10);

	ahrs = new AHRS(SPI::Port::kMXP); //also for te gyro
	ahrs->Reset();
if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		lift1->Set(.1);
		lift2->Set(.1);
	}
	else {
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
		}
	if(ahrs->GetAngle()>-89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);	//left side 50%
		srx3->Set(.5);

		srx4->Set(-.5);
		srx5->Set(-.5);	//right side 50%
		srx6->Set(-.5);
	}
	else if(-90.5 <= ahrs->GetAngle() and ahrs->GetAngle() <= -89.5 and srx1->GetSensorCollection().GetQuadraturePosition())	{
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
	else if (ahrs->GetAngle() < -90.5 and srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(-.2);
		srx2->Set(-.2);	//left side turn left 20% speed
		srx3->Set(-.2);

		srx4->Set(.2);
		srx5->Set(.2);	//right side turn left 20% speed
		srx6->Set(.2);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		intake1->Set(-.5);
		intake2->Set(-.5);
	}
}

auto4::auto4() {
	srx1 = new WPI_TalonSRX(1);
	srx2 = new WPI_TalonSRX(2);  //left drive
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right drive
	srx6 = new WPI_TalonSRX(6);

	intake1 = new WPI_TalonSRX(7); //intake
	intake2 = new WPI_TalonSRX(8);

	lift1 = new WPI_TalonSRX(9); //lift
	lift2 = new WPI_TalonSRX(10);

	ahrs = new AHRS(SPI::Port::kMXP); //also for te gyro
	ahrs->Reset();
if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		lift1->Set(.1);
		lift2->Set(.1);
	}
	else {
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
		}
	if(ahrs->GetAngle()>-89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);	//left side 50%
		srx3->Set(.5);

		srx4->Set(-.5);
		srx5->Set(-.5);	//right side 50%
		srx6->Set(-.5);
	}
	else if(-90.5 <= ahrs->GetAngle() and ahrs->GetAngle() <= -89.5 and srx1->GetSensorCollection().GetQuadraturePosition())	{
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
	else if (ahrs->GetAngle() < -90.5 and srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(-.2);
		srx2->Set(-.2);	//left side turn left 20% speed
		srx3->Set(-.2);

		srx4->Set(.2);
		srx5->Set(.2);	//right side turn left 20% speed
		srx6->Set(.2);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);
	}
	else if (srx1->GetSensorCollection().GetQuadraturePosition()) {
		intake1->Set(-.5);
		intake2->Set(-.5);
	}
}



