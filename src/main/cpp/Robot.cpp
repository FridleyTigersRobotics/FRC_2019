/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599704-driving-a-robot-using-mecanum-drive
#include <Robot.h>

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
  UpdateBallintake();
  UpdateSolenoid ();



void UpdateDriveSystem( void ) {
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


	void UpdateBallIntake( void ) {
		//ball intake
		bool xabutton = XboxController.GetAButton();
		bool xbbutton = XboxController.GetBButton();

		if( xabutton ) {
			mBallIntake.Set( -1 );
		}
		else if( xbbutton ) {
			mBallIntake.Set( 1 );
		}
		else {
 			mBallIntake.Set( 0 );
		}
	}

  void UpdateSolenoid (); {

    
  }

}

void Robot::TestPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
