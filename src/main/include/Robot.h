/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
	frc::VictorSP m_frontLeft{0};
  frc::VictorSP m_frontRight{1};		
	frc::VictorSP m_rearLeft{2}; 
	frc::VictorSP m_rearRight{3};
  frc::VictorSP m_frontLift{4};
	frc::VictorSP m_rearLift{5};
  frc::VictorSP m_ballSuck{6};
	frc::MecanumDrive m_drive{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
	frc::Joystick m_driveStick{1};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
