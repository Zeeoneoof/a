// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of 'this project.

#pragma once

#include <string>

#include <fmt/core.h>
#include <rev/SparkMax.h>
#include <frc/TimedRobot.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "Drivetrain.h"

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
  void SimulationInit() override;
  void SimulationPeriodic() override;
  frc::XboxController stick{0};

 private:
  frc::SendableChooser<std::string> m_chooser;
  frc::SendableChooser<std::string> field_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  /*rev::spark::SparkMax algaeMotor{29, rev::spark::SparkMax::MotorType::kBrushless};
  bool intakeAlgae = false;*/

  /*rev::spark::SparkMax intakeDeployMotor{29, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax intakeWheels{8, rev::spark::SparkMax::MotorType::kBrushless};

  frc::PIDController intakeDeployPID{0.1, 0, 0};

  rev::spark::SparkRelativeEncoder intakeEncoder = intakeDeployMotor.GetEncoder();*/

  frc::PIDController elevatorPID{0.1, 0, 0};

  /*rev::spark::SparkMax coralMotorHorizontal{14, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax coralMotorVertical{14, rev::spark::SparkMax::MotorType::kBrushless};
*/
  rev::spark::SparkMax elevatorMotor1{10, rev::spark::SparkMax::MotorType::kBrushless};
  //rev::spark::SparkMax elevatorMotor2{29, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder elevatorEncoder1 = elevatorMotor1.GetEncoder();
  //rev::spark::SparkRelativeEncoder elevatorEncoder2 = elevatorMotor2.GetEncoder();

  double x = 0;// left joystick x-axis
  double y = 0;// left joystick y-axis
  double x2 = 0;// right joystick x-axis
  float triggerL = 0;
  float triggerR = 0;
  bool FieldCentric = false;
};
