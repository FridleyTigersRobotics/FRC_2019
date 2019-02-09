/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <math.h>
#include <frc/IterativeRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <iostream>

//Robot specific includes
#include <frc/ADXRS450_Gyro.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>


// ***************************************************************************
//   Function:    deadband
//
//   Description: Applies a deadband to the input value.
//                (output) will be zero when (-zone) <= (value) <= (+zone)
//                (output) will be one at (value) == 1
//
//                         output        . (1,1)
//                            |         /
//                            |        /
//                            |       /
//                    -zone   |      /
//             ---------+-----+-----+--------- value
//                     /      |   +zone
//                    /       |
//                   /        |
//                  /         |
//                 . (-1,-1)
//
// ***************************************************************************
double deadband(
    double value,
    double zone
)
{
    double output;
    if ( fabs( value ) < zone )
    {
        output = 0.0;
    }
    else
    {
        // After the deadband, start with value = 0
        if( value < 0 )
        {
            output = ( value + zone ) / ( 1.0 - zone );
        }
        else
        {
            output = ( value - zone ) / ( 1.0 - zone );
        }
    }

    return output;
}




class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void Ball_intake( void )
    {
        frc::XboxController::JoystickHand const inHand  = frc::XboxController::JoystickHand::kRightHand;
        frc::XboxController::JoystickHand const outHand = frc::XboxController::JoystickHand::kLeftHand;

        // Retrieve the stick Y position
        double const inTriggerPosition  = XboxController.GetTriggerAxis( inHand  );
        double const outTriggerPosition = XboxController.GetTriggerAxis( outHand );

        // Apply deadband
        double const deadbandEnd       = 0.22;
        double const inTriggerPositionWithDeadband  = deadband( inTriggerPosition, deadbandEnd );
        double const outTriggerPositionWithDeadband = deadband( outTriggerPosition, deadbandEnd );

        // Calculate the motor speeds for the specified input
        double const m_ballIntake  = inTriggerPositionWithDeadband - outTriggerPositionWithDeadband;
    };
  void Tele_Lift(  void  )
    {
        bool const yButtonPressed = XboxController.GetYButton();
        bool const bButtonPressed = XboxController.GetBButton();
        bool const xButtonPressed = XboxController.GetXButton();
        bool const aButtonPressed = XboxController.GetAButton();

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
    };

 private:
    frc::VictorSP m_frontLeft{0};
    frc::VictorSP m_frontRight{1};		
	frc::VictorSP m_rearLeft{2}; 
	frc::VictorSP m_rearRight{3};
    frc::VictorSP m_frontLift{4};
	frc::VictorSP m_rearLift{5};
    frc::VictorSP m_ballIntake{6};
    frc::Solenoid wristSolenoid {0};
    frc::Solenoid pieceSolenoid {1};
    frc::DoubleSolenoid leftarmDouble {0, 1};
    frc::DoubleSolenoid rightarmDouble {2, 3};
    frc::MecanumDrive m_drive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};
	frc::Joystick m_driveStick{1};
    frc::ADXRS450_Gyro gyro{ frc::SPI::Port::kOnboardCS0  }; // Right turn -> positive angle
    frc::XboxController XboxController{0};
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
};
