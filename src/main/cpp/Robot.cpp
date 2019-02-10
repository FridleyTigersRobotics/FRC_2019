/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599704-driving-a-robot-using-mecanum-drive
#include <Robot.h>
#include <iostream>
// test

#include <frc/smartdashboard/SmartDashboard.h>



void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  //std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_drive.DriveCartesian(m_driveStick.GetX(), m_driveStick.GetY(), m_driveStick.GetZ());
  Tele_Lift();
  Ball_intake();
  UpdateDriveSystem();
}



void Robot::TestPeriodic() 
  {

  }


void Robot::Hatch_wrist( void )
{
    bool const yButtonPressed = XboxController.GetYButton();
    if ( yButtonPressed)
    {
        wristSolenoid.Set(true);
    }
    else
    {
        wristSolenoid.Set(false);
    }
    
}

void Robot::Ball_intake( void )
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
}


void Robot::Tele_Lift(  void  )
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
      };
  }

 
void Robot::UpdateDriveSystem( void ) {
        bool const debugDirectionChange = false;

		typedef enum {
		    waitForButtonPress,
		    waitForHoldTime,
		    waitForRelease,
            waitForReleaseTime,
		} debounceStateType;

		static debounceStateType debounceState = waitForButtonPress;
		static frc::Timer timer;
		bool buttonValue = XboxController.GetXButton();
		double holdTime = 0.03;
		double releaseTime = 0.25;
		static double driveMultiplier = 1.0;

        if ( debugDirectionChange )
        {
            char outputString[50];
            sprintf( outputString, "debounce: %10d, %10d, %10g\n", debounceState, buttonValue, timer.Get());
            std::cout << outputString;
            std::cout << std::flush;
        }

		switch ( debounceState )
		{
            case waitForButtonPress:
            {
                if ( buttonValue )
                {
                    timer.Reset();
                    timer.Start();
                    debounceState = waitForHoldTime;
                }
                break;
            }

            case waitForHoldTime:
            {
                if ( buttonValue )
                {
                    if ( timer.Get() > holdTime )
                    {
                        driveMultiplier *= -1.0;
                        debounceState = waitForRelease;
                    }
                }
                else
                {
                    debounceState = waitForButtonPress;
                    timer.Stop();
                }
                break;
            }

            case waitForRelease:
            {
                if ( buttonValue == false )
                {
                    timer.Reset();
                    timer.Start();
                    debounceState = waitForReleaseTime;
                }
                break;
            }

            case waitForReleaseTime:
            {
                if ( buttonValue == false )
                {
                    if ( timer.Get() > releaseTime )
                    {
                        debounceState = waitForButtonPress;
                    }
                }
                else
                {
                    debounceState = waitForRelease;
                    timer.Stop();
                }
                break;
            }

            default:
            {
                debounceState = waitForButtonPress;
                break;
            }

		}
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif




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

