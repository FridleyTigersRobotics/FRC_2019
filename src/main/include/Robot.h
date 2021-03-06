/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
// Basic
#include <math.h>
#include <stdio.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <iostream>

//Libraries
#include <frc/WPILib.h>
#include <frc/IterativeRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/TimedRobot.h>
#include <AHRS.h>


double deadband(
    double value,
    double zone
);


class Robot : public frc::TimedRobot {

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void UpdateDriveSystem ();
  void UpdateBallintake();
  void UpdateSolenoid ();
  void Hatch_wrist( void );
  void Ball_intake( void );
  void Tele_Lift(  void  );
  void Tele_FourBar(  void  );
  void Hatch_piece( void );

 private:
    // Motors
   	frc::VictorSP m_rearLeft{0}; 
	frc::VictorSP m_rearRight{1}; 
    frc::VictorSP m_frontLeft{2};
    frc::VictorSP m_frontRight{3};	
    frc::VictorSP m_ballIntake{4};
    frc::VictorSP m_frontLift{5};
	frc::VictorSP m_rearLift{6};
    // Solenoids
    frc::Solenoid wristSolenoid {1, 0};
    frc::Solenoid pieceSolenoid {1, 1};
    frc::DoubleSolenoid leftarmDouble {2, 0, 1};
    frc::DoubleSolenoid rightarmDouble {2, 2, 3};

    // Mechanum Drive
    frc::MecanumDrive m_drive{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
    // Game Controllers
	frc::Joystick buttonBoard{1};
    frc::XboxController XboxController{0};
    // Based on leg position
    frc::DigitalInput rearLimiterBot{ 4 };
    frc::DigitalInput frontLimiterBot{ 1 };
    frc::DigitalInput frontLimiterTop{ 0 };
    frc::DigitalInput rearLimiterTop{  3 };

    //frc::ADXRS450_Gyro gyro{ frc::SPI::Port::kOnboardCS0  }; // Right turn -> positive angle
    frc::AnalogInput FourBarPot{0};
    // Auto
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
    AHRS *ahrs;
    Compressor *c = new Compressor( 0 );
    
    frc::Timer time{};
};
