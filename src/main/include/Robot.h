// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot {
 public:
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

 private:
  SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  string m_autoSelected;

  // Controls
  Joystick rJoy = Joystick(0);

  // Components

  // DriveTrain
  frc::PWMVictorSPX frontLeft{1};
  frc::PWMVictorSPX rearLeft{2};
  frc::MotorControllerGroup left{frontLeft, rearLeft};

  frc::PWMVictorSPX frontRight{3};
  frc::PWMVictorSPX rearRight{4};
  frc::MotorControllerGroup right{frontRight, rearRight};

  frc::DifferentialDrive drive {left, right};
  // Values
  double joystickX;
  double joystickY;
};
