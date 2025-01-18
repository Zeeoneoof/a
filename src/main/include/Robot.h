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
  ctre::phoenix6::hardware::Pigeon2 Pigeon2{9};
  //applies smoothing to gyro
  double GyroValue = 0.0;
  double alpha = 1;
  double rawGyroValue = 0.0;
  double rotEr = 0;
  rev::spark::SparkMax intake{14, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax outtakeTilt{17, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax indexer{16, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax outtake{15, rev::spark::SparkMax::MotorType::kBrushless};
  frc::Compressor comp{19,frc::PneumaticsModuleType::REVPH};
  frc::DoubleSolenoid intakeSol{19,frc::PneumaticsModuleType::REVPH,5,7};

  rev::spark::SparkRelativeEncoder outtakeEnc = outtakeTilt.GetEncoder();
};
