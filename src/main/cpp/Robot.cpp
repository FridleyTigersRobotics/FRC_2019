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
            // AddDefault
  /* positionChooser.SetDefaultOption( "1) Nothing",  NO_POSITION );
  positionChooser.AddObject( "2) Left",  LEFT_POSITION );
  positionChooser.AddObject( "3) Middle", MIDDLE_POSITION );
  positionChooser.AddObject( "4) Right",  RIGHT_POSITION );
  frc::SmartDashboard::PutData( "Position Select", &positionChooser);

  actionChooser.AddDefault( "1) Do nothing", DO_NOTHING );
  actionChooser.AddObject( "2) Hatch", HATCH );
  actionChooser.AddObject( "3) Cargo", Cargo );
  frc::SmartDashboard::PutData( "Auto Action", &actionChooser);
  gyro.Calibrate();

*/
  CameraServer::GetInstance()->StartAutomaticCapture(); 
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

  if (positionChooser == middle && actionChooser == Hatch ) {
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
        pieceSolenoid.Set(true);
    }
    else
    {
        pieceSolenoid.Set(false);
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
    double m_ballIntakeSpeed = 0.0;
    m_ballIntake.Set( m_ballIntakeSpeed );
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
        m_ballIntakeSpeed = 0.0;
    }
    


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
      bool const buttonValue3 = buttonBoard.GetRawButton(3);
      bool const buttonValue4 = buttonBoard.GetRawButton(4);

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

    if ( buttonValue3 )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kForward;
    }
    else if ( buttonValue4 )
    {
      solenoidValue = frc::DoubleSolenoid::Value::kReverse;
    }

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


    frc::Timer driveTimer;
    frc::Timer liftTimer;
    frc::Timer cubeTimer;
    frc::Timer waitTimer;

    bool AutoCommandsFinished = false;

    double driveTimerEnd = 0.0;
    double liftTimerEnd  = 0.0;
    double cubeTimerEnd  = 0.0;
    double waitTimerEnd  = 0.0;

    bool driveCommandActive = false;
    bool liftCommandActive  = false;
    bool cubeCommandActive  = false;
    bool waitCommandActive  = false;
    bool turnCommandActive  = false;

    bool driveCommandBlocking = false;
    bool liftCommandBlocking  = false;
    bool cubeCommandBlocking  = false;
    bool waitCommandBlocking  = false;
    bool turnCommandBlocking  = false;


    drive_command_args_t currentDriveCommand;
    turn_command_args_t  currentTurnCommand;
    cube_command_args_t  currentCubeCommand;
    lift_command_args_t  currentLiftCommand;


    void Auto_Initalize( void )
    {
        gyro.Reset();
        currentCommandIndex = -1;
        AutoCommandsFinished = false;

        driveTimerEnd = 0.0;
        liftTimerEnd  = 0.0;
        cubeTimerEnd  = 0.0;
        waitTimerEnd  = 0.0;

        driveCommandActive = false;
        liftCommandActive  = false;
        cubeCommandActive  = false;
        waitCommandActive  = false;
        turnCommandActive  = false;

        driveCommandBlocking = false;
        liftCommandBlocking  = false;
        cubeCommandBlocking  = false;
        waitCommandBlocking  = false;
        turnCommandBlocking  = false;
        absoluteAngle = 0.0;
        maxAbsMotorChange = 1.0;
    }

    double absoluteAngle = 0.0;

    // ***************************************************************************
    //   Method:      Auto_ServiceCommandList
    //
    //   Description: Method for servicing the commands in the "AutoCommandList"
    //
    // ***************************************************************************
    void Auto_ServiceCommandList( void )
    {
        bool const debugCommandList = false;
        // Check for any active commands that need to be serviced.
        if ( driveCommandActive )
        {
            if ( driveTimer.Get() > driveTimerEnd )
            {
                Auto_StopDriving();
                driveCommandActive  = false;
                driveCommandBlocking = false;
            }
            else
            {
                Auto_DriveStraight(
                    currentDriveCommand.speed
                );
            }
        }

        if ( turnCommandActive )
        {
            bool turnCompleted = false;

            turnCompleted = \
            Auto_Turn(
                currentTurnCommand.speed,
                currentTurnCommand.angle
            );
            if ( turnCompleted == true )
            {
                turnCommandActive   = false;
                turnCommandBlocking = false;
            }
        }


        if ( !driveCommandActive && !turnCommandActive )
        {
            Auto_StopDriving();
        }

        if ( liftCommandActive )
        {
            if ( liftTimer.Get() > liftTimerEnd )
            {
                Auto_MoveLift(
                    HoldPosition
                );
                liftCommandActive   = false;
                liftCommandBlocking = false;
            }
            else
            {
                Auto_MoveLift(
                    currentLiftCommand.liftState
                );
            }
        }
        else
        {
            Auto_MoveLift(
                HoldPosition
            );
        }

        if ( cubeCommandActive )
        {
            if ( cubeTimer.Get() > cubeTimerEnd )
            {
                Auto_StopCube();
                cubeCommandActive   = false;
                cubeCommandBlocking = false;
            }
            else
            {
                Auto_MoveCargo(
                    currentCubeCommand.speed
                );
            }
        }

        if ( waitCommandActive )
        {
            if ( waitTimer.Get() > waitTimerEnd )
            {
                waitCommandActive  = false;
                waitCommandBlocking = false;
            }
            else
            {
                // do nothing
            }
        }

        if ( debugCommandList )
        {
            std::cout << "cmd ";
            std::cout << currentCommandIndex;
            std::cout << "dr_b ";
            std::cout << driveCommandBlocking;
            std::cout << " ";
            std::cout << driveCommandActive;
            std::cout << " lf_b ";
            std::cout << liftCommandBlocking;
            std::cout << " ";
            std::cout << liftCommandActive;
            std::cout << " cu_b ";
            std::cout << cubeCommandBlocking;
            std::cout << " ";
            std::cout << cubeCommandActive;
            std::cout << " wt_b ";
            std::cout << waitCommandBlocking;
            std::cout << " ";
            std::cout << waitCommandActive;
            std::cout << " tr_b ";
            std::cout << turnCommandBlocking;
            std::cout << " ";
            std::cout << turnCommandActive;
            std::cout << "\n";
            std::cout << std::flush;
        }


        // Service new command if we are not waiting on any blocking commands,
        // and we have not gotten to the end of the command list
        if ( !AutoCommandsFinished &&
             !driveCommandBlocking &&
             !liftCommandBlocking  &&
             !cubeCommandBlocking  &&
             !waitCommandBlocking  &&
             !turnCommandBlocking
            )
        {
            currentCommandIndex++;

            auto_command_t const *command = &(AutoCommandList[currentCommandIndex]);

            // Service current command
            switch ( command->commandType )
            {
                case DRIVE:
                {
                    Auto_ResetDriveDirection();
                    driveCommandActive   = true;
                    driveTimerEnd        = command->args.drive.time;
                    driveCommandBlocking = command->args.drive.blockCommands;
                    currentDriveCommand  = command->args.drive;

                    driveTimer.Reset();
                    driveTimer.Start();

                    break;
                }

                case LIFT:
                {
                    liftCommandActive   = true;
                    liftTimerEnd        = command->args.lift.time;
                    liftCommandBlocking = command->args.lift.blockCommands;
                    currentLiftCommand  = command->args.lift;

                    liftTimer.Reset();
                    liftTimer.Start();

                    break;
                }

                case CUBE:
                {
                    cubeCommandActive   = true;
                    cubeTimerEnd        = command->args.cube.time;
                    cubeCommandBlocking = command->args.cube.blockCommands;
                    currentCubeCommand  = command->args.cube;

                    cubeTimer.Reset();
                    cubeTimer.Start();

                    break;
                }

                case TURN:
                {
                    Auto_PrepareForTurn();
                    turnCommandActive   = true;
                    turnCommandBlocking = true;
                    currentTurnCommand  = command->args.turn;
                    absoluteAngle += command->args.turn.angle;
                    break;
                }

                case WAIT:
                {
                    waitCommandActive   = true;
                    waitTimerEnd        = command->args.wait.time;
                    waitCommandBlocking = true;
                    waitTimer.Reset();
                    waitTimer.Start();

                    break;
                }

                default:
                case FINISHED:
                {
                    AutoCommandsFinished = true;

                    break;
                }
            }
        }
    }





    typedef enum
    {
        notMoving,
        drivingForward,
        turning
    } auto_drive_state_t;

    auto_drive_state_t driveState = notMoving;
    double desiredDriveDirection = 0.0;


    // ***************************************************************************
    //   Method:      Auto_ResetDriveDirection
    //
    //   Description: Sets the robots current direction to be the direction that
    //                the Auto_DriveStraight function drives.
    //
    // ***************************************************************************
    void Auto_ResetDriveDirection( void )
    {
        desiredDriveDirection = gyro.GetAngle();//absoluteAngle;//gyro.GetAngle();
    }



    // ***************************************************************************
    //   Method:      Auto_ResetDriveDirection
    //
    //   Description: Function for Autonomous to keep the robot driving
    //                straight at the specified speed.
    //                Call Auto_ResetDriveDirection to initialize the "straight"
    //                direction.
    //
    // ***************************************************************************
    void Auto_DriveStraight(
        double speed
    )
    {
        bool const debugDriveStraight = false;
        double const currentDirection                     = gyro.GetAngle();
        double const directionError                       = desiredDriveDirection - currentDirection;
        double const directionErrorCompensationSaturation = 5;
        double const directionErrorSaturated              = fabs( saturate( directionError, -directionErrorCompensationSaturation, directionErrorCompensationSaturation ) );
        double const maxMotorSpeedCorrection              = 0.3;
        double const motorSpeedCorrection                 = maxMotorSpeedCorrection * ( directionErrorSaturated / directionErrorCompensationSaturation );

        // Set nominal speeds
        double leftMotorSpeed  = speed;
        double rightMotorSpeed = speed;

        if ( debugDriveStraight )
        {
            std::cout << "\ndes " << desiredDriveDirection;
            std::cout << " curr " << currentDirection;
            std::cout << " corr " << motorSpeedCorrection;
            std::cout << " mot " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        // Apply correction if needed.
        if ( directionError > 0 )
        {
            rightMotorSpeed = rightMotorSpeed - motorSpeedCorrection;
            if ( debugDriveStraight )
            {
                std::cout << " R ";
            }
        }
        else if ( directionError < 0 )
        {
            leftMotorSpeed = leftMotorSpeed - motorSpeedCorrection;
            if ( debugDriveStraight )
            {
                std::cout << " L ";
            }
        }
        else
        {
            // No correction needed.
            if ( debugDriveStraight )
            {
                std::cout << " N ";
            }
        }

        SetDriveMotorSpeeds(
            leftMotorSpeed,
            rightMotorSpeed
        );
    }


    // ***************************************************************************
    //   Method:      Auto_StopDriving
    //
    //   Description: Turns off the drive motors.
    //
    // ***************************************************************************
    void Auto_StopDriving( void )
    {
        SetDriveMotorSpeeds(
            0,
            0
        );
    }


    // ***************************************************************************
    //   Method:      Auto_MoveCargo
    //
    //   Description: Sets the carge intake to run at the specified speed.
    //
    // ***************************************************************************
    void Auto_MoveCargo(
        double speed
    )
    {
        SetCargoIntakeMotorSpeed(
            speed
        );
    }

    /*
    // ***************************************************************************
    //   Method:      Auto_StopCube
    //
    //   Description: Stops the cube intake motors.
    //
    // ***************************************************************************
    void Auto_StopCube( void )
    {
        SetCubeIntakeMotorSpeed(
            0.0
        );
    }
    

    // ***************************************************************************
    //   Method:      Auto_MoveLift
    //
    //   Description: Moves the cube lift at the specified speed.
    //
    // ***************************************************************************
    void Auto_MoveLift(
        lift_state_t liftState
    )
    {
         SetCubeLiftState(
             liftState,
             0.6
         );
    }


    double startingAngle   = 0.0;
    int    goodAngleCounts = 0;
    // ***************************************************************************
    //   Method:      Auto_PrepareForTurn
    //
    //   Description: Prepares for performing a turn with Auto_Turn.
    //                Right now just remembers the current angle so we know how
    //                much to turn.
    //
    // ***************************************************************************
    void Auto_PrepareForTurn( void )
    {
        std::cout << "\nStarting Angle: ";
        goodAngleCounts = 0;
        startingAngle = gyro.GetAngle(); //absoluteAngle;//
        std::cout << startingAngle;
    }



    // ***************************************************************************
    //   Method:      Auto_Turn
    //
    //   Description: Perform a turn at the specified speed to the specified angle.
    //                Auto_PrepareForTurn must be run before using this function
    //                to set the starting angle.
    //
    // ***************************************************************************
    bool Auto_Turn(
        double speed,
        double angle
    )
    {
        bool const debugTurn = false;
        bool turnCompleted = false;
        double const currentAngle = gyro.GetAngle();
        double const desiredAngle = startingAngle + angle;
        double const angleError   = currentAngle - desiredAngle;
        double leftMotorSpeed     = 0;
        double rightMotorSpeed    = 0;

        double const anglePrecision = 2.0;

        if ( debugTurn )
        {
            std::cout << "\nDesired Angle: " << desiredAngle;
            std::cout << "\nAngle Error: "   << angleError;
        }

        if ( fabs( angleError ) < anglePrecision )
        {
            goodAngleCounts++;
        }
        else
        {
            goodAngleCounts = 0;
        }


        if ( goodAngleCounts > 3 )
        {
            if ( debugTurn )
            {
                std::cout << "\nEnding Angle: " << currentAngle;
                std::cout << "\nEnding Error: " << angleError  << "\n";
            }
            turnCompleted = true;
        }
        else
        {
            double const speedMultiplier = saturate( angleError * 0.1, -1, 1 );

            leftMotorSpeed  = -speed * speedMultiplier;
            rightMotorSpeed =  speed * speedMultiplier;
        }

        if ( debugTurn )
        {
            std::cout << "\nmotor1: " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        // Set a minimum motor speed
        leftMotorSpeed = \
        minAbsLimitValue(
            leftMotorSpeed,
            0.2
        );

        rightMotorSpeed = \
        minAbsLimitValue(
            rightMotorSpeed,
            0.2
        );

        if ( debugTurn )
        {
            std::cout << "\nmotor2: " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        SetDriveMotorSpeeds(
            leftMotorSpeed,
            rightMotorSpeed
        );


        return turnCompleted;
    }