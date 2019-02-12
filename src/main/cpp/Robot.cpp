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
  // Enable these after testing them.
  //Tele_Lift();
  Tele_FourBar();
  //Ball_intake();
  UpdateDriveSystem();
  Hatch_wrist();
}



void Robot::TestPeriodic() 
  {

  }


void Robot::Hatch_wrist( void )
{
    bool const yButtonPressed = XboxController.GetYButton();
    if ( yButtonPressed )
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

       double m_frontLiftSpeed = 0.0;

       double m_rearLiftSpeed = 0.0;

       if ( xButtonPressed )
      {
          m_frontLiftSpeed = 1.0;
      }
      else if ( aButtonPressed )
      {
          m_frontLiftSpeed = -1.0;
      }
      else
      {
          m_frontLiftSpeed = 0.0;
      }


       if ( yButtonPressed )
      {
          m_rearLiftSpeed = 1.0;
      }
      else if ( bButtonPressed )
      {
          m_rearLiftSpeed = -1.0;
      }
      else
      {
          m_rearLiftSpeed = 0.0;
      };

      m_frontLift.Set( m_frontLiftSpeed );
      m_rearLift.Set( m_rearLiftSpeed );
  }




void Robot::Tele_FourBar(  void  )
  {
      bool const yButtonPressed = XboxController.GetYButton();
      bool const bButtonPressed = XboxController.GetBButton();
      bool const xButtonPressed = XboxController.GetXButton();
      bool const aButtonPressed = XboxController.GetAButton();
      static int numValues = 0;
      static long sum = 0;

     numValues++;
     sum += FourBarPot.GetValue();

     if ( numValues > 20 )
     {
       double const avg = (double)sum / (double)numValues;
        std::cout << avg << "\n" << std::flush;
        numValues = 0;
        sum = 0;
     }



      frc::DoubleSolenoid::Value solenoidValue = frc::DoubleSolenoid::Value::kOff;

    if ( 0 )
    {
      std::cout << m_rearLeft.Get() << " "
      << m_rearRight.Get() << " "
      << m_frontLeft.Get() << " "
      << m_frontRight.Get()	<< " " 
      << "\n" << std::flush;
    }

    if ( aButtonPressed )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kForward;
    }
    else if ( bButtonPressed )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kReverse;
    }

    leftarmDouble.Set(  solenoidValue );
    rightarmDouble.Set( solenoidValue );
  }
 
void Robot::UpdateDriveSystem( void ) {
  frc::XboxController::JoystickHand const driveStickHand = frc::XboxController::JoystickHand::kLeftHand;
  frc::XboxController::JoystickHand const turnStickHand = frc::XboxController::JoystickHand::kRightHand;
  
  double const DriveHandX = XboxController.GetX( driveStickHand );
  double const DriveHandY = XboxController.GetX( driveStickHand );
  double const TurnHandX  = XboxController.GetX( turnStickHand );
  double const deadbandSize = 0.2;
  double const xSpeed    = deadband( DriveHandX, deadbandSize );
  double const ySpeed    = deadband( DriveHandY, deadbandSize );
  double const turnSpeed = deadband( TurnHandX,  deadbandSize );

  //m_drive.DriveCartesian( xSpeed, ySpeed, turnSpeed );
   m_drive.DriveCartesian( 0, 0, 0 );
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

