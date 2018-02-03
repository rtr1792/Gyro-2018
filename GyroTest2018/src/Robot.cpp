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
	AHRS *ahrs;
	frc::Talon frontLeft{1};
	frc::Talon midLeft{2};
	frc::Talon rearLeft{3};
	frc::SpeedControllerGroup left{frontLeft, midLeft, rearLeft};
	frc::Talon frontRight{4};
	frc::Talon midRight{5};
	frc::Talon rearRight{6};
	frc::SpeedControllerGroup right{frontRight, midRight, rearRight};
	frc::DifferentialDrive drive{left, right};
	double rotation;

	frc::DifferentialDrive DifferentialDrive { left, right };
	void RobotInit() {

	}
	void AutonomousInit() override {

	}
	void AutonomousPeriodic() {

	}
	void TeleopInit() {

	}
	void TeleopPeriodic() {
	rotation = ahrs->GetYaw();
	}
	void TestPeriodic() {
		frc::SmartDashboard::PutNumber("Vertical_Rotation", rotation);
		frc::SmartDashboard::PutNumber("Name", 2.0);
	}
private:
	frc::Joystick stick {0};
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
START_ROBOT_CLASS(Robot)
