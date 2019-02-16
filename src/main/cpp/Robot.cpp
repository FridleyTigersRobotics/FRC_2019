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

int SumtoPot(int fourBarSum )
    {
        if (fourBarSum == 0)
        {
            return 1053;
        }
        else if (fourBarSum == 1)
        {
            return 1150;
        }

        else if (fourBarSum == 2) 
        {
            return 1300;
        }
        else if (fourBarSum == 3) 
        {
            return 1500;
        }
        else if (fourBarSum == 4) 
        {
            return 1700;
        }
        else if (fourBarSum == 5) 
        {
            return 2700;
        }
        else if (fourBarSum == 6) 
        {
            return 3000;
        }
                
        else
        {
            return 1054;
        }
    }

void Robot::RobotInit() {
        /*    // AddDefault
  positionChooser.SetDefaultOption( "1) Nothing",  NO_POSITION );
  positionChooser.AddObject( "2) Left",  LEFT_POSITION );
  positionChooser.AddObject( "3) Middle", MIDDLE_POSITION );
  positionChooser.AddObject( "4) Right",  RIGHT_POSITION );
  frc::SmartDashboard::PutData( "Position Select", &positionChooser);

  actionChooser.AddDefault( "1) Do nothing", DO_NOTHING );
  actionChooser.AddObject( "2) Hatch", HATCH );
  actionChooser.AddObject( "3) Cargo", Cargo );
  frc::SmartDashboard::PutData( "Auto Action", &actionChooser);

  CameraServer::GetInstance()->StartAutomaticCapture(); */
  ahrs = new AHRS(SPI::Port::kMXP);
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
  //  std::cout << time.Get() << "\n";
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

void Robot::TeleopInit() {

    time.Start();
}

void Robot::TeleopPeriodic() {
  // Enable these after testing them.
  Tele_Lift();
  Tele_FourBar();
  Ball_intake();
  UpdateDriveSystem();
  Hatch_wrist();
  Hatch_piece();
}



void Robot::TestPeriodic() 
  {

  }


void Robot::Hatch_wrist( void )
{
    bool const buttonValue2 = buttonBoard.GetRawButton(2);
    /*
TODO
toggle


    */
    if ( buttonValue2 )
    {
        wristSolenoid.Set(true);
    }
    else
    {
        wristSolenoid.Set(false);
    }
    
}

void Robot::Hatch_piece( void )
{
    bool const buttonValue1 = buttonBoard.GetRawButton(1);
    if ( buttonValue1 )
    {
        pieceSolenoid.Set(false);
    }
    else
    {
        pieceSolenoid.Set(true);
    }
    
}

void Robot::Ball_intake( void )
{
    /*frc::XboxController::JoystickHand const inHand  = frc::XboxController::JoystickHand::kRightHand;
    frc::XboxController::JoystickHand const outHand = frc::XboxController::JoystickHand::kLeftHand;

    // Retrieve the stick Y position
    double const inTriggerPosition  = XboxController.GetTriggerAxis( inHand  );
    double const outTriggerPosition = XboxController.GetTriggerAxis( outHand );

    // Apply deadband
    double const deadbandEnd       = 0.22;
    double const inTriggerPositionWithDeadband  = deadband( inTriggerPosition, deadbandEnd );
    double const outTriggerPositionWithDeadband = deadband( outTriggerPosition, deadbandEnd );

    // Calculate the motor speeds for the specified input
    double const m_ballIntake  = inTriggerPositionWithDeadband - outTriggerPositionWithDeadband;*/
    double m_ballIntakeSpeed=0.15;
    bool const buttonValue5 = buttonBoard.GetRawButton(5);
    bool const buttonValue6 = buttonBoard.GetRawButton(6);
    
    if ( buttonValue5 )
    {
        m_ballIntakeSpeed = 1.0;
    }
    else if (buttonValue6) 
    {
        m_ballIntakeSpeed = -1.0;
    }

    else
    {
        m_ballIntakeSpeed = 0.15;
    }
    m_ballIntake.Set( m_ballIntakeSpeed );


}






typedef enum
{
    Waiting,
    LiftUp,
    LiftDown,
    TiltForward
} lift_state_t;

lift_state_t liftState = Waiting;


void Robot::Tele_Lift(  void  )
  {
      bool const yButtonPressed = XboxController.GetYButton(); // up
      bool const bButtonPressed = XboxController.GetBButton(); // tilt
      //bool const xButtonPressed = XboxController.GetXButton();
      bool const aButtonPressed = XboxController.GetAButton(); // down
      bool const buttonValue10 = buttonBoard.GetRawButton(10); // down
      bool const buttonValue9 = buttonBoard.GetRawButton(9); // down

       //bool const notAtTopLimit = winchLimiterTop.Get(); // Value of the limiter is nominally one, and zero when limit is hit.
      //bool const notAtBotLimit = winchLimiterBot.Get(); // Value of the limiter is nominally one, and zero when limit is hit.

       double m_frontLiftSpeed = 0.0;

       double m_rearLiftSpeed = 0.0;

    //std::cout << "pitch " << ahrs->GetPitch() << "\n";
    //std::cout << "roll " << ahrs->GetRoll() << "\n";
    //std::cout << "yaw " << ahrs->GetYaw() << "\n";

      if ( buttonValue10 )
      {
          liftState = TiltForward;
      }
      else if ( buttonValue9 )
      {
          liftState = LiftUp;
      }
      else if ( aButtonPressed )
      {
          liftState = LiftDown;
      }
      else
      {
          liftState = Waiting;
      }

      if ( liftState == LiftUp )
      {
          double const desiredAngle = 2.5;
          double const maxSpeed = 0.9;
          double const maxAngle = 15.0;
          double angle = ahrs->GetRoll();
          double angleError = angle - desiredAngle;
          m_frontLiftSpeed = maxSpeed;
          m_rearLiftSpeed  = maxSpeed;

          if ( angleError > 0 )
          {
              // Tilting backwards, slow front motor
              double motorSpeedDiff    = angleError / maxAngle;
              double motorSpeedDiffSat = ( motorSpeedDiff > maxSpeed ) ? maxSpeed : motorSpeedDiff; 

              m_frontLiftSpeed -= motorSpeedDiffSat;
          }
          else
          {
              // Tilting forwards, slow rear motor
              double motorSpeedDiff    = -angleError / maxAngle;
              double motorSpeedDiffSat = ( motorSpeedDiff > maxSpeed ) ? maxSpeed : motorSpeedDiff; 

              m_rearLiftSpeed -= motorSpeedDiffSat;
          }

      }
      else if ( liftState == TiltForward )
      {
          m_frontLiftSpeed = -1.0;
          m_rearLiftSpeed = 0.0;



      }
      else if ( liftState == LiftDown )
      {
          m_frontLiftSpeed = -1.0;
          m_rearLiftSpeed  = -1.0;
      }
      else // waiting
      {
          m_frontLiftSpeed = 0.0;
          m_rearLiftSpeed  = 0.0;
      }

      m_frontLift.Set( -m_frontLiftSpeed );
      m_rearLift.Set( m_rearLiftSpeed );
  }




void Robot::Tele_FourBar(  void  )
  {
      bool const buttonValue3 = buttonBoard.GetRawButton(3);
      bool const buttonValue4 = buttonBoard.GetRawButton(4);
      double const fourBarShift = buttonBoard.GetRawAxis(1);
      std::cout <<"4bar "<< fourBarShift << "\n";
      
      static int fourBarSum = 0;

      static int numValues = 0;
      static long sum = 0;
    
      static int buttonCount = 0;
      static double lastButtonValue  = 0;

      std::cout <<"4barsum "<< fourBarSum << "\n";


    if (lastButtonValue == fourBarShift)
    {
        if ( buttonCount < 10 )
        {
            buttonCount += 1;
        }   
    }
    else
    {
        buttonCount = 0;
        lastButtonValue = fourBarShift;
    }

    if (buttonCount == 6)
    {
      if (fourBarShift == 1)
      {
          if (fourBarSum < 5)
            {
                fourBarSum += 1;
            }

      }
      if (fourBarShift == -1)
      {
          if (fourBarSum > 0)
            {
                fourBarSum -= 1;
            }

      }
    }


     numValues++;
     int potValue = FourBarPot.GetValue();
     sum += potValue;
     std::cout << "pot"  << potValue << "\n";

     if ( numValues > 20 )
     {
       double const avg = (double)sum / (double)numValues;
        //std::cout << avg << "\n" << std::flush;
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
    double const errorThreshold = 250;
    double fourBarError;
    fourBarError = potValue - SumtoPot(fourBarSum);

    if (fourBarError > errorThreshold)
    {
        solenoidValue = frc::DoubleSolenoid::Value::kReverse;
    }
    if (fourBarError < -errorThreshold)
    {
        solenoidValue = frc::DoubleSolenoid::Value::kForward;
    }    






/*
    if ( buttonValue3 )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kForward;
    }
    else if ( buttonValue4 )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kReverse;
    }
*/
    leftarmDouble.Set(  solenoidValue );
    rightarmDouble.Set( solenoidValue );
  }
 
void Robot::UpdateDriveSystem( void ) {
  frc::XboxController::JoystickHand const driveStickHand = frc::XboxController::JoystickHand::kLeftHand;
  frc::XboxController::JoystickHand const turnStickHand = frc::XboxController::JoystickHand::kRightHand;
  
  double const DriveHandY = XboxController.GetX( driveStickHand );
  double const DriveHandX = XboxController.GetY( driveStickHand );
  double const TurnHandX  = XboxController.GetX( turnStickHand );
  double const deadbandSize = 0.2;
  double const xSpeed    = -deadband( DriveHandX, deadbandSize );
  double const ySpeed    = deadband( DriveHandY, deadbandSize );
  double const turnSpeed = deadband( TurnHandX,  deadbandSize );

   m_drive.DriveCartesian( xSpeed, ySpeed, turnSpeed );
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

