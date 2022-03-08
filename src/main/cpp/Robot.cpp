// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <cmath>
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

static const int INVERT_LEFT = 1;
static const int INVERT_RIGHT = 1;
static const double JOYSTICK_THRESH = 0.05;

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

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
  //forces the CANSparkMax motors to stop and await orders
  rearLeft.StopMotor();
  frontLeft.StopMotor();
  rearRight.StopMotor();
  frontRight.StopMotor();

  //links the back motors to the front motors in movement terms
  rearLeft.Follow(frontLeft);
  rearRight.Follow(frontRight);
  currentDistance = 0;
  

  //frontRight.GetForwardLimitSwitch()

  //meant to set he values for the mapping of the firing mechanism
  firingValues.insert(pair<int, float>(2, 0.75));
  firingValues.insert(pair<int, float>(3, 1.00));
  firingValues.insert(pair<int, float>(4, 0.25));
  firingValues.insert(pair<int, float>(5, 0.50));
}

void Robot::TeleopPeriodic() {
  if(typeOfDrive)ArcDrv();else TankDrv();
  if(lJoy->GetRawButtonPressed(11))typeOfDrive = !typeOfDrive;
  //FireButtons();
  if(lJoy->GetRawButtonPressed(7)) ControlArm();
}

//checks the buttons to see the rate of fire
void Robot::FireButtons(){
  for(int i = 2; i<=5; i++)
    if(rJoy->GetRawButtonPressed(i)) fireMotor.Set(firingValues.at(i)/5.0);
}

//Control over the arm
void Robot::ControlArm(){

  //Mandatory pause to allow for change in control
  for(int i = 0; i<100; i++);

  //Sets the mode over to Arcade Drive to still allow for maneuverability
  typeOfDrive = true;
  armEngage = true;

  while(armEngage){
    joystickArm = rJoy->GetY();
    if(abs(joystickArm) > JOYSTICK_THRESH && currentDistance <= 100 && currentDistance >= 0){
      armMotor.Set(joystickArm/5.0);
      if(joystickArm > 0){currentDistance++;}
      else if(joystickArm < 0){currentDistance--;}
    } 
    
    //allows for another button press to revert everything back to normal
    if(lJoy->GetRawButtonPressed(7)){for(int i = 0; i<100; i++); armEngage = false; typeOfDrive=false;}
  }
}

void Robot::ArcDrv(){
  joystickX = lJoy->GetX();
  joystickY = lJoy->GetY();
  
  if(abs(joystickX) < JOYSTICK_THRESH) joystickX = 0;
  if(abs(joystickY) < JOYSTICK_THRESH) joystickY = 0;

  drive.ArcadeDrive(-1*joystickX/1.5, joystickY/1.5); // Squared inputs true by default
}
void Robot::TankDrv(){
  joystickX = lJoy->GetY();
  joystickY = rJoy->GetY();

  if(abs(joystickX) < JOYSTICK_THRESH) joystickX = 0;
  if(abs(joystickY) < JOYSTICK_THRESH) joystickY = 0;

  drive.TankDrive(joystickX/1.5, -1*joystickY/1.5); // Squared inputs true by default
}

void Robot::DisabledInit() {
  drive.StopMotor();
}

void Robot::DisabledPeriodic() {
  drive.StopMotor();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
