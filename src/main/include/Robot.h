// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/GenericHID.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
/*Took this out and worked fine, but not sure if we need this in the future
#include <frc/motorcontrol/MotorController.h>*/
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/MotorControllerGroup.h>


using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot {
 public:
  //Declaring all the functions/methods used
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void ArcDrv();
  void TankDrv();

 private:
  SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  string m_autoSelected;

  // Controls
  frc::Joystick* lJoy = new frc::Joystick{0};
  frc::Joystick* rJoy = new frc::Joystick{1};
  frc2::Button b_lJoy = frc2::JoystickButton(lJoy, 11);// tankButton = JoystickButton(rJoy, 0);
  // Components

  // DriveTrain
  
  rev::CANSparkMax frontLeft{1, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rearLeft{2, rev::CANSparkMax::MotorType::kBrushed};

  rev::CANSparkMax frontRight{3, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rearRight{4, rev::CANSparkMax::MotorType::kBrushed};
  

  DifferentialDrive drive {frontLeft, frontRight};

  // Values
  //https://software-metadata.revrobotics.com/REVLib.json
  double joystickX;
  double joystickY;
  bool check = true;
};
