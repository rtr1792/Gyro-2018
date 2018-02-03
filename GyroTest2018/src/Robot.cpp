#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Joystick.h>
#include <AHRS.h>

class Robot : public frc::IterativeRobot {
public:
	AHRS *gyro;

	frc::Talon frontLeft{1};
	frc::Talon midLeft{2};
	frc::Talon rearLeft{3};
	frc::SpeedControllerGroup left{frontLeft, midLeft, rearLeft};
	frc::Talon frontRight{4};
	frc::Talon midRight{5};
	frc::Talon rearRight{6};
	frc::SpeedControllerGroup right{frontRight, midRight, rearRight};
	frc::DifferentialDrive drive{left, right};
	//float rotation;
	frc::DifferentialDrive DifferentialDrive { left, right };

	void RobotInit() {
		gyro = new AHRS(SPI::Port::kMXP);
	}
	void AutonomousInit() override {

	}
	void AutonomousPeriodic() {

	}
	void TeleopInit() {

	}
	void TeleopPeriodic() {
//	double angle = gyro->GetAngle();
		if (xbox.GetRawButton(1)){
			 gyro->Reset();
		}
		frc::SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
		frc::SmartDashboard::PutNumber("Name", 2.0);
	}
	void TestPeriodic() {

	}
private:
	frc::Joystick stick {0};
	frc::XboxController xbox {1};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
START_ROBOT_CLASS(Robot)
