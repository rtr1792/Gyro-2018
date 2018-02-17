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
#include "math.h"
#include <Timer.h>

//Crosses the baseline, nothing more
AutoManager::AutoManager() {
	srx1 = new WPI_TalonSRX(1);
	srx2 = new WPI_TalonSRX(2);  //left drive
	srx3 = new WPI_TalonSRX(3);

	srx4 = new WPI_TalonSRX(4);
	srx5 = new WPI_TalonSRX(5);  //right drive
	srx6 = new WPI_TalonSRX(6);

	intake1 = new WPI_TalonSRX(7); //intake
	intake2 = new WPI_TalonSRX(8);

	liftSRX1 = new WPI_TalonSRX(9); //lift
	liftSRX2 = new WPI_TalonSRX(10);

	pi = 3.141592653589793238462643383279502884;
	constant = 1024/pi;

	timer = new Timer();

	ahrs = new AHRS(SPI::Port::kMXP); //also for ta gyro
	ahrs->Reset();


//srx1->GetSensorCollection().SetQuadraturePosition(0,100);
}

void AutoManager::Auto1() {
frc::SmartDashboard::PutNumber("encoder rotation", srx1->GetSensorCollection().GetQuadraturePosition());

//frc::SmartDashboard::PutNumber("time since start", timer->Get());

frc::SmartDashboard::PutNumber("time since start", timer->Get());

	timer->Start();
	timer->Get();
//after 15 seconds, reset the timer to zero
if(timer->Get() >= 15) {
		timer->Reset();
		srx1->GetSensorCollection().SetQuadraturePosition(0,0);
  }
// run the lift until 2 seconds have passed
if(timer->Get() <= 2) {
	liftSRX1->Set(.3);
	liftSRX2->Set(.3);
}
else {
	liftSRX1->Set(0);
	liftSRX2->Set(0);
}
// go forward from 2 to 5 seconds
if((srx1->GetSensorCollection().GetQuadraturePosition() < 190 * constant) and timer->Get() > 2 and timer->Get() < 5 ) {
	srx1->Set(.1);
	srx2->Set(.1);  //left side set to 10% speed
	srx3->Set(.1);

	srx4->Set(.1);
	srx5->Set(.1);  //right side set to 10% speed
	srx6->Set(.1);
  }
	else {
	srx1->Set(0);
	srx2->Set(0);  //left side stop
	srx3->Set(0);

	srx4->Set(0);
	srx5->Set(0);  //right side stop
	srx6->Set(0);
	}
//turn right from 5 seconds and beyond
if(timer->Get() > 5 ) {
	srx1->Set(.1);
	srx2->Set(.1);  //left side set to 10% speed
	srx3->Set(.1);

	srx4->Set(-.1);
	srx5->Set(-.1);  //right side set to 10% speed
	srx6->Set(-.1);
  }
	else {
	srx1->Set(0);
	srx2->Set(0);  //left side stop
	srx3->Set(0);

	srx4->Set(0);
	srx5->Set(0);  //right side stop
	srx6->Set(0);
	}



}

//starts on left side, delivers cube to switch when on the same side
void AutoManager::Auto2() {

if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		liftSRX1->Set(.1);
		liftSRX2->Set(.1);
}
	else {
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
	}
if(ahrs->GetAngle() < 89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
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

//start in rightmost position, deliver cube to same side
void AutoManager::Auto3() {

if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		liftSRX1->Set(.1);
		liftSRX2->Set(.1);
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

//start in middle, deliver to right side of switch
void AutoManager::Auto4() {

if(srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);

		liftSRX1->Set(.1);
		liftSRX2->Set(.1);
	}
	else {
		srx1->Set(0);
		srx2->Set(0);	//left side stop
		srx3->Set(0);

		srx4->Set(0);
		srx5->Set(0);	//right side stop
		srx6->Set(0);
		}
	if(ahrs->GetAngle() < 89.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
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
	else if(ahrs->GetAngle() > 0 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(-.5);
		srx2->Set(-.5);  //left side set to 50% speed
		srx3->Set(-.5);

		srx4->Set(.5);
		srx5->Set(.5);  //right side set to 50% speed
		srx6->Set(.5);
	}
	else if(ahrs->GetAngle() < -0.5 and srx1->GetSensorCollection().GetQuadraturePosition()){
		srx1->Set(.5);
		srx2->Set(.5);  //left side set to 50% speed
		srx3->Set(.5);

		srx4->Set(-.5);
		srx5->Set(-.5);  //right side set to 50% speed
		srx6->Set(-.5);
	}
	else if(srx1->GetSensorCollection().GetQuadraturePosition()){
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



