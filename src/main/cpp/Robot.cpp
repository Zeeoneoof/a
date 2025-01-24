// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

float x = 0;// left joystick x-axis
float y = 0;// left joystick y-axis
float x2 = 0;// right joystick x-axis
float triggerL = 0;
float triggerR = 0;
bool FieldCentric = false;

Drivetrain drivetrain2;

void Robot::RobotInit() {
  // Code executed when the robot is initialized
  frc::SmartDashboard::PutNumber("Straight P", 0.02);
  frc::SmartDashboard::PutNumber("Straight I", 0.0);
  frc::SmartDashboard::PutNumber("Not Straight D", 0.0);

  // Setting default options for autonomous mode
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  field_chooser.SetDefaultOption("FieldCentric", "FieldCentric");
  field_chooser.AddOption("FieldCentric","FieldCentric");
  field_chooser.AddOption("RobotCentric","RobotCentric");
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Center", &field_chooser);
}

void Robot::RobotPeriodic() {
  // Code executed periodically during teleop mode
  if (stick.GetRawButtonPressed(8) == true){
    Pigeon2.Reset();
  }
  double rawangle = Pigeon2.GetYaw().GetValueAsDouble();
  rawGyroValue = double(fmod((rawangle + 360.0), 360.0));  // Replace this with your actual gyro reading
  // Apply the low-pass filter
  GyroValue = alpha * rawGyroValue + (1.0 - alpha) * GyroValue;
  GyroValue = 0;
}

void Robot::AutonomousInit() {
  // Code executed when autonomous mode is initialized
  FieldCentric = false;
  // Selecting autonomous mode
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

}

void Robot::AutonomousPeriodic() {
    drivetrain2.Update(x, y, x2, GyroValue, triggerL, triggerR,FieldCentric);
  // Code executed periodically during autonomous mode
  if (m_autoSelected == kAutoNameCustom) {
   
 
  } else if (m_autoSelected == kAutoNameDefault) {
    
  }
}

void Robot::TeleopInit() {
  // Code executed when teleop mode is initialized
  FieldCentric = false;
}

void Robot::TeleopPeriodic() {
  // Code executed periodically during teleop mode
  
  // Get joystick inputs
  float rawx = stick.GetRawAxis(0);
  float rawy = stick.GetRawAxis(1);
  float rawx2 = stick.GetRawAxis(4);
  triggerL = -stick.GetRawAxis(2)*0.2;
  triggerR = stick.GetRawAxis(3)*0.3;
  bool bumperR = stick.GetRawButton(6);
  bool bumperL = stick.GetRawButton(5);
  bool A = stick.GetRawButton(1);
  bool B = stick.GetRawButtonPressed(2);
  int dpad = stick.GetPOV(0);

  
  // Algae manipulator
  // Every time the A button is pressed, the motor spins in the opposite direction
  bool spongebob = true;
    if (A && spongebob) {
      algaeMotor.Set(-0.6);
      spongebob = false;
    }
    else if (A && !spongebob) {
      algaeMotor.Set(0.6);
      spongebob = true;
    }
    else {
    algaeMotor.Set(0);
  }

// Intake deploy motor
// If B is pressed, move to setpoint
  int setpoint = 30;
  float position = intakeEncoder.GetPosition();
  if (B) {
    while (position < setpoint) {
      intakeDeployMotor.Set(0.6);
    }
    while (position > setpoint) {
      intakeDeployMotor.Set(-0.6);
    }
  } else {
    intakeDeployMotor.Set(0);
  }



  if ((rawx >= 0.1)||(rawx <= -0.1)){x = (rawx);} 
  else {x=0;}
  if ((rawy >= 0.1)||(rawy <= -0.1)){y = -(rawy);}
  else {y=0;}
  if ((rawx2>= 0.2)||(rawx2<= -0.2)){x2=rawx2*2;}
  else {x2=0;}

  if (field_chooser.GetSelected()=="FieldCentric"){
    FieldCentric = true;
  } else if (field_chooser.GetSelected()=="RobotCentric") {
    FieldCentric = false;
  }

  drivetrain2.Update(x, y, x2, GyroValue, triggerL, triggerR,FieldCentric);

  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {

drivetrain2.Update(x, y, x2, GyroValue, triggerL, triggerR,FieldCentric);

}
void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif