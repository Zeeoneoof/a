// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Drivetrain m_swerve;

void Robot::RobotInit() {
  //intakeEncoder.SetPosition(0);
  elevatorEncoder1.SetPosition(0);
  //elevatorEncoder2.SetPosition(0);
}

void Robot::RobotPeriodic() {
  // Code executed periodically during teleop mode
 
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

  if (m_autoSelected == kAutoNameCustom) {
 
  } else if (m_autoSelected == kAutoNameDefault) {

  }
}

void Robot::TeleopInit() {
  //fmt::print("A value", "running");
  frc::SmartDashboard::PutString("running", "running");
  FieldCentric = true;
}

void Robot::TeleopPeriodic() {
  // Algae Manipulator
  //Runs wheels when button held, toggles Direction each time button pressed
  /*bool A = stick.GetRawButton(1);
  bool a = stick.GetRawButtonReleased(1);
  if (a){intakeAlgae = !intakeAlgae;}
  algaeMotor.Set(A ? (intakeAlgae ? 0.15 : -0.15) : 0);*/

  // Intake deploy and run motor
  /*bool B = stick.GetRawButton(2);
  // If B is pressed, move to setpoint until the intake wheels have resistance.
  int intakeSetpoint = 10;
  float position = intakeEncoder.GetPosition();
  float intakeWheelAmps = intakeWheels.GetOutputCurrent();
  // Deploy motor code
  while (B) {
  //when button held deploy out to setpoint and run intakeWheels while checking for amperage
    position = intakeEncoder.GetPosition();
    intakeWheelAmps = intakeWheels.GetOutputCurrent();
    intakeDeployMotor.Set(std::clamp(intakeDeployPID.Calculate(position, intakeSetpoint), -0.1, 0.1));
    intakeWheels.Set(0.1);
    frc::SmartDashboard::PutNumber("position", position);
    frc::SmartDashboard::PutNumber("intakeWheelAmps", intakeWheelAmps);
    if (intakeWheelAmps > 3) {
      break;
    }
  }
  //when button not held revieve back in to 0 in the robot
  if (position > 2) {
    intakeDeployMotor.Set(std::clamp(intakeDeployPID.Calculate(position, 0), -0.1, 0.1));
    position = intakeEncoder.GetPosition();
    frc::SmartDashboard::PutNumber("position", position);
  } else {
    intakeDeployMotor.Set(0);
  }
  intakeWheels.Set(0);*/

  //Elevator
  int dpad = stick.GetPOV(0);
  frc::SmartDashboard::PutNumber("dpad", dpad);
  if (dpad != -1) {
    dpad = stick.GetPOV(0);
    frc::SmartDashboard::PutNumber("dpad", dpad);
    switch (dpad) {
    case (0):
      elevatorMotor1.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder1.GetPosition(), 0), -0.2, 0.3));
      //elevatorMotor2.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder2.GetPosition(), 0), -0.1, 0.1));
      frc::SmartDashboard::PutNumber("elevatorPos1", elevatorEncoder1.GetPosition());
      //frc::SmartDashboard::PutNumber("elevatorPos2", elevatorEncoder2.GetPosition());
      frc::SmartDashboard::PutString("elevator", "working");
      break;
    case (90):
      elevatorMotor1.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder1.GetPosition(), 40), -0.2, 0.3));
      //elevatorMotor2.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder2.GetPosition(), -10), -0.1, 0.1));
      break;
    case (180):
      elevatorMotor1.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder1.GetPosition(), 80), -0.2, 0.3));
      //elevatorMotor2.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder2.GetPosition(), -20), -0.1, 0.1));
      break;
    case (270):
      elevatorMotor1.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder1.GetPosition(), 160), -0.2, 0.3));
      //elevatorMotor2.Set(std::clamp(elevatorPID.Calculate(elevatorEncoder2.GetPosition(), -30), -0.1, 0.1));
      break;
    }
    frc::SmartDashboard::PutNumber("elevatorPos1", elevatorEncoder1.GetPosition());
    //frc::SmartDashboard::PutNumber("elevatorPos2", elevatorEncoder2.GetPosition());
  } else {
    elevatorMotor1.Set(0);
  }

   // Get joystick inputs
  float rawx = stick.GetRawAxis(0);
  float rawy = stick.GetRawAxis(1);
  float rawx2 = stick.GetRawAxis(4);

  // Joystick Deadzones
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

  m_swerve.Update(x, y, x2, 0, triggerL, triggerR,FieldCentric);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif