// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <thread>
#include <cstdio>

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
#include <frc/AnalogInput.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<frc/DigitalInput.h>


using namespace frc;
using namespace std;

class Robot : public TimedRobot {
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
  void FireButtons();
  void ControlArm();
  void Shooting();
  void CAM();
  //static void VisionThread();

 private:
  SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  string m_autoSelected;

  // Controls
  frc::Joystick* lJoy = new frc::Joystick{0};
  frc::Joystick* rJoy = new frc::Joystick{1};
  // TODO: Awful variable name maybe toggleTank?
  // No, it makes sense "button_leftJoystick" makes sense >:(
    
  //frc2::Button b_lJoy = frc2::JoystickButton(lJoy, 11);// tankButton = JoystickButton(rJoy, 0);
  
  // Components
  frc::AnalogInput stopTheArm{0};
  frc::AnalogInput stopTheTurn{1};

  // DriveTrain
  
  rev::CANSparkMax shooter{7, rev::CANSparkMax::MotorType::kBrushed};

  rev::CANSparkMax frontLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeft{2, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax frontRight{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRight{4, rev::CANSparkMax::MotorType::kBrushless};

  DifferentialDrive drive {frontLeft, frontRight};

  //Firing Motors
  rev::CANSparkMax cam {5, rev::CANSparkMax::MotorType::kBrushed};
  // map<int, float> firingValues; // Why use a map here? It allowed for a better button mapping system

  //Arm Motor(s)
  rev::CANSparkMax armMotor{6, rev::CANSparkMax::MotorType::kBrushed};
  DigitalInput limitSwitchClimber{0}; 
  DigitalInput limitSwitchCAMSystem{1};
  
  // Servo(s)
  frc::Servo firstServo {1};

  // Timer(s)
  frc::Timer firstTimer = Timer();

  // Values
  //https://software-metadata.revrobotics.com/REVLib.json
  double joystickX;
  double joystickY;
  double joystickArm;
  double currentDistance;
  bool typeOfDrive = true;
  bool armEngage = false;
  int shooterCounter = 0; 
  double weight;
  const double JOYSTICK_THRESH = 0.05;
  //Vision
  //cs::UsbCamera camera;
};