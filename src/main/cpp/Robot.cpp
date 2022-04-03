// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <cmath>
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>



static const int INVERT_LEFT = 1;
static const int INVERT_RIGHT = 1;

// Makes the controller less sensitive to joystick


//magical thread video
static void VisionThread(){
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetResolution(640,480);
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("CameraToUse",640,480);
  cv::Mat mat;
  while(true){
    if(cvSink.GrabFrame(mat) == 0){
      outputStream.NotifyError(cvSink.GetError());
      continue;
    }
    outputStream.PutFrame(mat);
  }
}

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  std::thread visionThread(VisionThread);
  visionThread.detach();
}



/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
  
  //m_autoSelected = m_chooser.GetSelected();
  //m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  // TODO: use output streams (<<)
  //fmt::print("Auto selected: {}\n", m_autoSelected);

  
  if (m_autoSelected == kAutoNameCustom) {
    //  Custom Auto goes here
  } else {
    firstTimer.Reset();
    firstTimer.Start();
    // Default Auto goes here
  }
}

// need to determine the distance to travel
// Autonomous lasts 15 sec
// tank ( neg, pos ) -> moves forwards
// tank ( pos, neg ) -> moves backwards
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    if (double(firstTimer.Get()) < 5) {
      drive.TankDrive(0.5, -0.5);
    } else {
      drive.TankDrive(0.0, 0.0);
    }
    // Default Auto goes here
  }
}

void Robot::TeleopInit(){
  // forces the CANSparkMax motors to stop and await orders
  rearLeft.StopMotor();
  frontLeft.StopMotor();
  rearRight.StopMotor();
  frontRight.StopMotor();

  // links the back motors to the front motors in movement terms
  rearLeft.Follow(frontLeft);
  rearRight.Follow(frontRight);
 

  // frontRight.GetForwardLimitSwitch()

  // meant to set the values for the mapping of the firing mechanism
  // firingValues.insert(pair<int, float>(2, 0.75));
  // firingValues.insert(pair<int, float>(3, 1.00));
  // firingValues.insert(pair<int, float>(4, 0.25));
  // firingValues.insert(pair<int, float>(5, 0.50));

  // sets the servo position to 0
  // firstServo.Set(0);
}

void Robot::TeleopPeriodic(){
  //shooting section of calls
  if (typeOfDrive){ArcDrv();}
  else{TankDrv();}
  if (lJoy->GetRawButtonPressed(11)){typeOfDrive = !typeOfDrive;}
  
  //SmartDashboard::PutNumber("before Shooting", -1);
  Shooting();
  //SmartDashboard::PutNumber("before Control Arm", -2);
  ControlArm();
  //SmartDashboard::PutNumber("before CAM", -3);
  CAM();
}

//this should under understandable pretenses allow the turning motor for the arm
//to be forced to go a set distance
//90 was chosen as an arbitrary value for now as to determine limits of the motor
void Robot::CAM(){
  SmartDashboard::PutBoolean("ljoyGetRawButton 3", lJoy->GetRawButton(3));
  if(rJoy->GetRawButton(4)) {
    cam.Set(0.15);
    currentDistance++;
  }
  else if(rJoy->GetRawButton(5)) {
    cam.Set(-0.15);
    currentDistance--;
  }
  else{
    cam.Set(0.0);
  }
}

// checks to see if any of the firing buttons are pressed, and set the firing rate to its correseponding
// --- To Scrap Later ---
void Robot::FireButtons(){
  //firstServo.Set(0);
  // for (int i = 2; i <= 5; i++)
  //   if (rJoy->GetRawButtonPressed(i))
  //     fireMotor.Set(firingValues.at(i) / 5.0);
  /*
  if(rJoy->GetRawButtonPressed(2)){
    fireMotor.Set(firingValues.at(2)/5.0);
  }else if(rJoy->GetRawButtonPressed(3)){
    fireMotor.Set(firingValues.at(3)/5.0);
  }else if(rJoy->GetRawButtonPressed(4)){ 
    fireMotor.Set(firingValues.at(4)/5.0);
  }else if(rJoy->GetRawButtonPressed(5)){
    fireMotor.Set(firingValues.at(5)/5.0);
  }else{
    fireMotor.Set(0);
  }
  */
}

// START OF RANDOM CODE
void Robot::Shooting(){
  /*
  if (lJoy->GetRawButton(2) && shooterCounter == 0){
    firstServo.Set(1.0);
    shooter.Set(-1);
    shooterCounter++;
  }
  else{
    firstServo.Set(0);
    shooter.Set(0);
    shooterCounter = 0;
  }*/
}
// END OF RANDOM CODE

// Control over the arm (for hanging I believe)
void Robot::ControlArm(){
  SmartDashboard::PutBoolean("LimitSwitch", limitSwitchClimber.Get());
  SmartDashboard::PutBoolean("rjoyGetRawButton 2", rJoy->GetRawButton(2));
  // checks to see if the arm is too far either out or in
  if (!limitSwitchClimber.Get()){
    armMotor.StopMotor();
    if (rJoy->GetRawButton(3)){
      armMotor.Set(-0.2);
    }
  }
  else{
    // lets the driver give physical, analog input
    if (rJoy->GetTrigger()){
      joystickArm = rJoy->GetY();
      if (abs(joystickArm) > JOYSTICK_THRESH){
        armMotor.Set(joystickArm * 0.95 );
      }
    }
    // meant to make the arm go forward
    else if (rJoy->GetRawButton(2)){
      armMotor.Set(0.60);
    }
    // meant to make the arm go backward
    else if (rJoy->GetRawButton(3)){
      armMotor.Set(-0.60);
    }
    // stops the arm from moving at all
    else{
      armMotor.Set(0);
    }
  }
}

void Robot::ArcDrv(){
  // Read the joystick for X and Y values
  joystickX = lJoy->GetX();
  joystickY = lJoy->GetY();

  SmartDashboard::PutNumber("joystick x inital 1:", 1);

  // Stops the motors from moving if the joystick is too little or stops unintentional movements
  if (abs(joystickX) < JOYSTICK_THRESH)
    joystickX = 0;
  if (abs(joystickY) < JOYSTICK_THRESH)
    joystickY = 0;

  
  //calculation for weight ( determines level of linear ctrl vs radical ctrl )
  //weight = (1/2)*(lJoy->GetThrottle())+0.5;

  // equation to make this thing move faster forward and backwards
  //joystickY = ((weight) * pow(joystickY, 1.8)) + ((1 - weight) * joystickY);
  //joystickX = ((weight) * pow(joystickX, 1.8)) + ((1 - weight) * joystickX);

  //SmartDashboard::PutNumber("jY", jY);
  SmartDashboard::PutNumber("Joystick X", joystickX);
  SmartDashboard::PutNumber("Joystick Y", joystickY);

  drive.ArcadeDrive(joystickX, -1*joystickY); // Squared inputs true by default

  SmartDashboard::PutNumber("AfterAdrive", 4);
}


void Robot::TankDrv(){
  // reads from both joysticks for x and y
  joystickX = lJoy->GetY();
  joystickY = rJoy->GetY();

  // Stops the motors from moving if the joystick is too little or stops unintentional movements
  if (abs(joystickX) < JOYSTICK_THRESH)
    joystickX = 0;
  if (abs(joystickY) < JOYSTICK_THRESH)
    joystickY = 0;

  // to reenable if Kirwan asks for it ( for what ever reason, if he needs it I guess? )
  // calculation for weight ( determines level of linear ctrl vs radical ctrl )
  //weight = lJoy->GetRawAxis(2);
  //joystickY = (0.5 * pow(joystickY, 1.8)) + (0.5 * joystickY);
  //joystickX = (0.5 * pow(joystickX, 1.8)) + (0.5 * joystickX);

  drive.TankDrive(-1*joystickX/1.5, joystickY/1.5); // Squared inputs true by default
}

void Robot::DisabledInit()
{
  drive.StopMotor();
}

void Robot::DisabledPeriodic()
{
  drive.StopMotor();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
