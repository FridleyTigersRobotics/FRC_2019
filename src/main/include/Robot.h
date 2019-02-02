/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>

#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <Spark.h>
#include <Timer.h>

//Robot specific includes
#include <Timer.h>
#include <ADXRS450_Gyro.h>
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
  void Tele_Lift() ;
    {
        bool const yButtonPressed = XboxController.GetYButton( );
        bool const bButtonPressed = XboxController.GetBButton( );
        bool const xButtonPressed = XboxController.GetXButton( );
        bool const aButtonPressed = XboxController.GetAButton( );

        //bool const notAtTopLimit = winchLimiterTop.Get(); // Value of the limiter is nominally one, and zero when limit is hit.
        //bool const notAtBotLimit = winchLimiterBot.Get(); // Value of the limiter is nominally one, and zero when limit is hit.

        double m_frontLift = 0.0;

        double m_rearLift = 0.0;

        if ( xButtonPressed )
        {
            m_frontLift = 1.0;
        }
        else if ( aButtonPressed )
        {
            m_frontLift = -1.0;
        }
        else
        {
            m_frontLift = 0.0;
        }

        if ( yButtonPressed )
        {
            m_rearLift = 1.0;
        }
        else if ( bButtonPressed )
        {
            m_rearLift = -1.0;
        }
        else
        {
            m_rearLift = 0.0;
        }
    }

 private:
	frc::VictorSP m_frontLeft{7};
  frc::VictorSP m_frontRight{1};		
	frc::VictorSP m_rearLeft{2}; 
	frc::VictorSP m_rearRight{3};
  frc::VictorSP m_frontLift{4};
	frc::VictorSP m_rearLift{5};
  frc::VictorSP m_ballSuck{6};
	frc::MecanumDrive m_drive{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
	frc::Joystick m_driveStick{1};
  frc::ADXRS450_Gyro gyro{ frc::SPI::Port::kOnboardCS0  }; // Right turn -> positive angle
  frc::XboxController XboxController{ 0 };

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
