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

int SumtoPot(int fourBarSum)
{
    if (fourBarSum == 0)
    {
        // ground
        return 1012;
    }
    else if (fourBarSum == 1)
    {
        return 1300;
    }

    else if (fourBarSum == 2)
    {
        // cargo level 1
        return 1728;
    }
    else if (fourBarSum == 3)
    {
        // Rocket hatch level 2
        return 2070;
    }
    else if (fourBarSum == 4)
    {
        return 3043;
    }
    else if (fourBarSum == 5)
    {
        return 2700;
    }
    else if (fourBarSum == 6)
    {
        //Hatch level 3 / max
        return 3043;
    }

    else
    {
        return 1054;
    }
}

void Robot::RobotInit()
{
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
*/
  CameraServer::GetInstance()->StartAutomaticCapture( 0 ); 
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
void Robot::RobotPeriodic()
{
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
void Robot::AutonomousInit()
{
 //   m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString(
    //     "Auto Selector", kAutoNameDefault);
    //std::cout << "Auto selected: " << m_autoSelected << std::endl;
/*
  if ( startingPosition == LEFT_POSITION &&
             autoAction       == LOAD_HATCH   &&
            )
        {
            AutoCommandList = FromLeftLoadHatch;
        }
        */
}
 


void Robot::AutonomousPeriodic()
{
    //TeleopPeriodic();
    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::TeleopInit()
{

    time.Start();
}

void Robot::TeleopPeriodic()
{
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

void Robot::Hatch_wrist(void)
{
    bool const buttonValue2 = buttonBoard.GetRawButton(2);
    const int buttonPressCountLimit1 = 3;
    static int buttonPressCount1 = 0;
    static int wristToggle = 0;

    if (buttonValue2)
    {
        if ( buttonPressCount1 <= buttonPressCountLimit1 )
        {
            buttonPressCount1++;
        }

        if ( buttonPressCount1 == buttonPressCountLimit1 )
        {
            if (wristToggle == 0)
            {
                wristToggle = 1;
            }
            else
            { 
                wristToggle = 0;
            }
        }
    }
    else
    {
        buttonPressCount1 = 0;
    }
    

    if ( wristToggle == 1 )
    {
        wristSolenoid.Set(true);
    }
    else
    {
        wristSolenoid.Set(false);
    }



}

void Robot::Hatch_piece(void)
{
    bool const buttonValue1 = buttonBoard.GetRawButton(1);
    const int buttonPressCountLimit = 3;
    static int buttonPressCount = 0;
    static int pieceToggle = 0;

    if (buttonValue1)
    {
        if ( buttonPressCount <= buttonPressCountLimit )
        {
            buttonPressCount++;
        }

        if ( buttonPressCount == buttonPressCountLimit )
        {
            if (pieceToggle == 0)
            {
                pieceToggle = 1;
            }
            else
            { 
                pieceToggle = 0;
            }
        }
    }
    else
    {
        buttonPressCount = 0;
    }
    

    if ( pieceToggle == 1 )
    {
        pieceSolenoid.Set(true);
    }
    else
    {
        pieceSolenoid.Set(false);
    }



}

void Robot::Ball_intake(void)
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
    double m_ballIntakeSpeed = 0.15;
    bool const buttonValue5 = buttonBoard.GetRawButton(5);
    bool const buttonValue6 = buttonBoard.GetRawButton(6);

    if (buttonValue5)
    {
        m_ballIntakeSpeed = -0.5;
    }
    else if (buttonValue6)
    {
        m_ballIntakeSpeed = 0.5;
    }

    else
    {
        m_ballIntakeSpeed = -0.2;
    }
    m_ballIntake.Set(m_ballIntakeSpeed);
}

typedef enum
{
    Waiting,
    LiftUp,
    LiftDown,
    TiltForward
} lift_state_t;

lift_state_t liftState = Waiting;

void Robot::Tele_Lift(void)
{
    bool const yButtonPressed = XboxController.GetYButton(); // up
    bool const bButtonPressed = XboxController.GetBButton(); // tilt
    bool const xButtonPressed = XboxController.GetXButton();
    bool const aButtonPressed = XboxController.GetAButton(); // down
    bool const buttonValue10 = buttonBoard.GetRawButton(10); // down
    bool const buttonValue9 = buttonBoard.GetRawButton(9);   // down

    if (1)
    {
        if (yButtonPressed)
        {
            m_frontLift.Set(0.8);
        }
        else if (bButtonPressed)
        {
            m_frontLift.Set(-0.8);
        }
        else
        {
            m_frontLift.Set(0.0);
        }

        if (xButtonPressed)
        {
            m_rearLift.Set(-0.8);
        }
        else if (aButtonPressed)
        {
            m_rearLift.Set(0.8);
        }
        else
        {
            m_rearLift.Set(0.0);
        }
    }
    else
    {

        //bool const notAtTopLimit = winchLimiterTop.Get(); // Value of the limiter is nominally one, and zero when limit is hit.
        //bool const notAtBotLimit = winchLimiterBot.Get(); // Value of the limiter is nominally one, and zero when limit is hit.

        double m_frontLiftSpeed = 0.0;

        double m_rearLiftSpeed = 0.0;

        //std::cout << "pitch " << ahrs->GetPitch() << "\n";
        //std::cout << "roll " << ahrs->GetRoll() << "\n";
        //std::cout << "yaw " << ahrs->GetYaw() << "\n";

        if (buttonValue10)
        {
            liftState = TiltForward;
        }
        else if (buttonValue9)
        {
            liftState = LiftUp;
        }
        else if (aButtonPressed)
        {
            liftState = LiftDown;
        }
        else
        {
            liftState = Waiting;
        }

        if (liftState == LiftUp)
        {
            double const desiredAngle = 2.5;
            double const maxSpeed = 0.9;
            double const maxAngle = 15.0;
            double angle = ahrs->GetRoll();
            double angleError = angle - desiredAngle;
            m_frontLiftSpeed = maxSpeed;
            m_rearLiftSpeed = maxSpeed;

            if (angleError > 0)
            {
                // Tilting backwards, slow front motor
                double motorSpeedDiff = angleError / maxAngle;
                double motorSpeedDiffSat = (motorSpeedDiff > maxSpeed) ? maxSpeed : motorSpeedDiff;

                m_frontLiftSpeed -= motorSpeedDiffSat;
            }
            else
            {
                // Tilting forwards, slow rear motor
                double motorSpeedDiff = -angleError / maxAngle;
                double motorSpeedDiffSat = (motorSpeedDiff > maxSpeed) ? maxSpeed : motorSpeedDiff;

                m_rearLiftSpeed -= motorSpeedDiffSat;
            }
        }
        else if (liftState == TiltForward)
        {
            m_frontLiftSpeed = -1.0;
            m_rearLiftSpeed = 0.0;
        }
        else if (liftState == LiftDown)
        {
            m_frontLiftSpeed = -1.0;
            m_rearLiftSpeed = -1.0;
        }
        else // waiting
        {
            m_frontLiftSpeed = 0.0;
            m_rearLiftSpeed = 0.0;
        }

        m_frontLift.Set(-m_frontLiftSpeed);
        m_rearLift.Set(m_rearLiftSpeed);
    }
}

void Robot::Tele_FourBar(void)
{
    if (0)
    {
        frc::DoubleSolenoid::Value solenoidValue = frc::DoubleSolenoid::Value::kOff;

        static bool lastWasUp = false;
        const int offCounterUpMax = 5;
        const int offCounterDownMax = 0;
        static int offCounter = 0;

        int const buttonCountThreshold = 3;
        int const numButtons = 4;
        int static buttonCounts[numButtons] = {0};
        static bool TurnOffSolenoids = true;
        bool handleButtonPress[numButtons] = {false};
        bool handleButtonRelease[numButtons] = {false};

        for (int button = 0; button < numButtons; button++)
        {
            bool const buttonValue = buttonBoard.GetRawButton(button + 1);

            if (buttonValue)
            {
                if (buttonCounts[button] <= buttonCountThreshold)
                {
                    buttonCounts[button]++;
                }
            }
            else
            {
                if (buttonCounts[button] >= buttonCountThreshold)
                {
                    handleButtonPress[button] = false;
                    handleButtonRelease[button] = true;
                }
                buttonCounts[button] = 0;
            }

            if (buttonCounts[button] == buttonCountThreshold)
            {
                handleButtonPress[button] = true;
                handleButtonRelease[button] = false;
            }
        }

        if (TurnOffSolenoids == true)
        {
            offCounter++;

            if ((lastWasUp && (offCounter > offCounterUpMax)) ||
                (!lastWasUp && (offCounter > offCounterDownMax)))
            {
                std::cout << "off" << offCounter << "\n";
                leftarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
                rightarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
                offCounter = 0;
                TurnOffSolenoids = false;
            }
        }

        if (handleButtonPress[0])
        {
            std::cout << "forward\n";
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            TurnOffSolenoids = false;
        }
        else if (handleButtonPress[1])
        {
            std::cout << "reverse\n";
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
            TurnOffSolenoids = false;
        }
        else if (handleButtonPress[2])
        {
            std::cout << "forward\n";
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            TurnOffSolenoids = true;
            lastWasUp = true;
        }
        else if (handleButtonPress[3])
        {
            std::cout << "reverse\n";
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
            TurnOffSolenoids = true;
            lastWasUp = false;
        }
    }
    else
    {
        frc::DoubleSolenoid::Value solenoidValue = frc::DoubleSolenoid::Value::kOff;
        bool const buttonValue3 = buttonBoard.GetRawButton(3);
        bool const buttonValue4 = buttonBoard.GetRawButton(4);
        double const fourBarShift = buttonBoard.GetRawAxis(1);

        static int fourBarSum = 0;

        static int numValues = 0;
        static long sum = 0;

        static int buttonCount = 0;
        static double lastButtonValue = 0;

        if (lastButtonValue == fourBarShift)
        {
            if (buttonCount < 10)
            {
                buttonCount += 1;
            }
        }
        else
        {
            buttonCount = 0;
            lastButtonValue = fourBarShift;
        }

        if (buttonCount == 3)
        {
            if (fourBarShift == 1)
            {
                if (fourBarSum < 6)
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
        //std::cout << "pot"  << potValue << "\n";
        //std::cout << FourBarPot.GetValue() << "\n";
        if (numValues > 50)
        {
            double const avg = (double)sum / (double)numValues;
            //std::cout << avg << "\n";
            numValues = 0;
            sum = 0;
        }

        if (0)
        {
            std::cout << m_rearLeft.Get() << " "
                      << m_rearRight.Get() << " "
                      << m_frontLeft.Get() << " "
                      << m_frontRight.Get() << " "
                      << "\n"
                      << std::flush;
        }

        static double lastPotValue = 0;
        double const errorThreshold = 250;
        int fourBarError;
        fourBarError = potValue - SumtoPot(fourBarSum);
        double fourBarDeriv = potValue - lastPotValue;

        static int Index = 0;
        int const numHistValues = 5;
        static int PotValueHist[numHistValues] = {0};

        static bool currentSolenoidValuesInit = false;
        static frc::DoubleSolenoid::Value currentSolenoidValues[numHistValues];

        if (currentSolenoidValuesInit == false)
        {
            currentSolenoidValuesInit = true;
            for (int idx = 0; idx < numHistValues; idx++)
            {
                currentSolenoidValues[idx] = frc::DoubleSolenoid::Value::kOff;
            }
        }

        static uint32_t DutyCycleHistIndex = 0;
        const uint32_t DutyCycleHistLen = 5;

        static int64_t errorIntegral = 0;
        solenoidValue = currentSolenoidValues[Index];

        PotValueHist[Index] = potValue;

        errorIntegral += fourBarError;

        double Kp = 0.006;
        double Ki = 0.00001;
        double Kd = 0.01;

        static int LastError = 0;

        int errorDerivative = LastError - fourBarError;
        static double errorDerivativeSum = 0.0;
        static double errorSum = 0.0;
        static double errorIntegralSum = 0.0;
        static double PositionSum = 0.0;

        errorSum += fourBarError;
        errorDerivativeSum += errorDerivative;
        errorIntegralSum += errorIntegral;
        PositionSum += potValue;
        LastError = fourBarError;

        if (Index == (numHistValues - 1))
        {
            double errorDerivativeAvg = errorDerivativeSum / numHistValues;
            double errorAvg = errorSum / numHistValues;
            double errorIntegralAvg = errorIntegralSum / numHistValues;
            double PositionAvg = PositionSum / numHistValues;

            frc::DoubleSolenoid::Value dir;

            //double DutyCycleDbl = Kp * (double)errorAvg + Ki * (double)errorIntegralAvg + Kd * (double)errorDerivativeAvg;
            double DutyCycleDbl = Kp * (double)fourBarError + Ki * (double)errorIntegral + Kd * (double)errorDerivative;
            //int DutyCycle = floor( abs( DutyCycleDbl ) );
            int DutyCycle = (DutyCycleDbl > 0) ? (std::min(floor(abs(DutyCycleDbl)), 2.0)) : (floor(abs(DutyCycleDbl)));

            std::cout
                << Index << " "
                << fourBarSum << " "
                << solenoidValue << " "
                << SumtoPot(fourBarSum) << " "
                << errorIntegralAvg << " "
                << errorDerivativeAvg << " "
                << DutyCycleDbl << " "
                << DutyCycle * ((DutyCycleDbl < 0) ? (-1) : (1)) << " "
                << Kp * (double)fourBarError << " "
                << Ki * (double)errorIntegral << " "
                << Kd * (double)errorDerivative << " "
                                                   "\n";

            errorDerivativeSum = 0.0;
            errorSum = 0.0;
            errorIntegralSum = 0.0;
            PositionSum = 0.0;

            if (DutyCycleDbl > 0)
            {
                dir = frc::DoubleSolenoid::Value::kReverse;
            }
            else
            {
                dir = frc::DoubleSolenoid::Value::kForward;
            }

            for (int idx = 0; idx < numHistValues; idx++)
            {
                if (idx < DutyCycle)
                {
                    currentSolenoidValues[idx] = dir;
                }
                else
                {
                    currentSolenoidValues[idx] = frc::DoubleSolenoid::Value::kOff;
                }
            }

            Index = 0;
        }
        else
        {
            Index++;
        }

        if (solenoidValue == frc::DoubleSolenoid::Value::kForward)
        {
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
        }
        else if (solenoidValue == frc::DoubleSolenoid::Value::kReverse)
        {
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        {
            leftarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
            rightarmDouble.Set(frc::DoubleSolenoid::Value::kReverse);
        }
    }

    // override with big cylinder
    //if(buttonValue3)
    //solenoidValue = frc::DoubleSolenoid::Value::kForward;
    //if(buttonValue4)solenoidValue = frc::DoubleSolenoid::Value::kReverse;

    //end override

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
}

void Robot::UpdateDriveSystem(void)
{
    frc::XboxController::JoystickHand const driveStickHand = frc::XboxController::JoystickHand::kLeftHand;
    frc::XboxController::JoystickHand const turnStickHand = frc::XboxController::JoystickHand::kRightHand;

    double const DriveHandY = XboxController.GetX(driveStickHand);
    double const DriveHandX = XboxController.GetY(driveStickHand);
    double const TurnHandX = XboxController.GetX(turnStickHand);
    double const deadbandSize = 0.2;
    double const xSpeed = -deadband(DriveHandX, deadbandSize);
    double const ySpeed = deadband(DriveHandY, deadbandSize);
    double const turnSpeed = deadband(TurnHandX, deadbandSize);

    m_drive.DriveCartesian(xSpeed, ySpeed, turnSpeed);
}






#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
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
    double zone)
{
    double output;
    if (fabs(value) < zone)
    {
        output = 0.0;
    }
    else
    {
        // After the deadband, start with value = 0
        if (value < 0)
        {
            output = (value + zone) / (1.0 - zone);
        }
        else
        {
            output = (value - zone) / (1.0 - zone);
        }
    }

    return output;
}
