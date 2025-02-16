#include <frc/smartdashboard/SmartDashboard.h>

#include "Drivetrain.h"

Drivetrain::Drivetrain () {

    //Drive Krakens configurations
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(1);
    driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    m_FL_Drive.GetConfigurator().Apply(driveConfig);
    m_FR_Drive.GetConfigurator().Apply(driveConfig);
    m_BL_Drive.GetConfigurator().Apply(driveConfig);
    m_BR_Drive.GetConfigurator().Apply(driveConfig);

    //Steer Krakens configurations // also fusing CANcoder to Krakens encoder for higher accuracy
    //configFL.Feedback.FeedbackRemoteSensorID = CANcoderFL.GetDeviceID();
    //configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    //configFL.Feedback.SensorToMechanismRatio = 1.0;
    //configFL.Feedback.RotorToSensorRatio = 18;
    configFL.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(1);
    configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    //configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
    //configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    //configFR.Feedback.SensorToMechanismRatio = 1.0;
    //configFR.Feedback.RotorToSensorRatio = 18;
    configFR.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(1);
    configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    //configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
    //configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    //configBL.Feedback.SensorToMechanismRatio = 1.0;
    //configBL.Feedback.RotorToSensorRatio = 18;
    configBL.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(1);
    configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    //configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
    //configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    //configBR.Feedback.SensorToMechanismRatio = 1.0;
    //configBR.Feedback.RotorToSensorRatio = 18;
    configBR.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(1);
    configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    m_FL_Steer.GetConfigurator().Apply(configFL);
    m_FR_Steer.GetConfigurator().Apply(configFR);
    m_BL_Steer.GetConfigurator().Apply(configBL);
    m_BR_Steer.GetConfigurator().Apply(configBR);

    //Configure CANcoder to zero the magnet appropriately
    /*cc_cfg.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0.4_tr;

    CANcoderFL.GetConfigurator().Apply(cc_cfg);
    CANcoderFR.GetConfigurator().Apply(cc_cfg);
    CANcoderBL.GetConfigurator().Apply(cc_cfg);
    CANcoderBR.GetConfigurator().Apply(cc_cfg);*/

    /*
    music.Restart
    music.AddInstrument(m_FL_Drive);
    music.AddInstrument(m_FL_Steer);
    music.AddInstrument(m_BL_Drive);
    music.AddInstrument(m_BL_Steer);
    music.AddInstrument(m_FR_Drive);
    music.AddInstrument(m_FR_Steer);
    music.AddInstrument(m_BR_Drive);
    music.AddInstrument(m_BR_Steer);
    music.LoadMusic("imperial.chrp");
    music.Play();
    music.LoadMusic("output.chrp");
    music.Play();
    */
}

void Drivetrain::Update (double x, double y, double x2, double GyroValue, double triggerL, double triggerR, bool FieldCentric)  {

  straightP = frc::SmartDashboard::GetNumber("Straight P", 0.02);
  straightI = frc::SmartDashboard::GetNumber("Straight I", 0);
  straightD = frc::SmartDashboard::GetNumber("Not Straight D", 0);

  //get wheel angle with motor encoder
  /*double FL_pos = std::fmod(M_PI/180*(18*double(m_FL_Steer.GetPosition().GetValue())), 2*M_PI);
  if (FL_pos < 0) FL_pos += 2*M_PI;
  double FR_pos = std::fmod(M_PI/180*(18*double(m_FR_Steer.GetPosition().GetValue())), 2*M_PI);
  if (FR_pos < 0) FR_pos += 2*M_PI;
  double BL_pos = std::fmod(M_PI/180*(18*double(m_BL_Steer.GetPosition().GetValue())), 2*M_PI);
  if (BL_pos < 0) BL_pos += 2*M_PI;
  double BR_pos = std::fmod(M_PI/180*(18*double(m_BR_Steer.GetPosition().GetValue())), 2*M_PI);
  if (BR_pos < 0) BR_pos += 2*M_PI;*/

  //get wheel angle with CANcoder
  /*double FL_pos = (m_FL_Steer.GetPosition().GetValueAsDouble());
  double FR_pos = (m_FR_Steer.GetPosition().GetValueAsDouble());
  double BL_pos = (m_BL_Steer.GetPosition().GetValueAsDouble());
  double BR_pos = (m_BR_Steer.GetPosition().GetValueAsDouble());*/

  double FL_pos = CANcoderFL.GetAbsolutePosition().GetValueAsDouble()*(2*M_PI);
  double FR_pos = CANcoderFR.GetAbsolutePosition().GetValueAsDouble()*(2*M_PI);
  double BL_pos = CANcoderBL.GetAbsolutePosition().GetValueAsDouble()*(2*M_PI);
  double BR_pos = CANcoderBR.GetAbsolutePosition().GetValueAsDouble()*(2*M_PI);
  
  frc::SmartDashboard::PutNumber("BL_Angle",BL_pos);


  //Calculate time difference since last loop
  units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
  units::time::second_t deltaTime = currentTime - lastTime;
  double doubleDeltaTime = double(deltaTime);
  lastTime = currentTime;

  //double R = sqrt((L*L)+(W*W));
  double STR = x;
  double FWD = y;
  double iError;
  double lastError;

  double temp;
  if (FieldCentric == true) {
    temp = FWD*cos(GyroValue*M_PI/180) + STR*sin(GyroValue*M_PI/180);
    STR =-FWD*sin(GyroValue*M_PI/180) + STR*cos(GyroValue*M_PI/180);
    FWD = temp;
  } else {
    STR = x;
    FWD = y;
  }

  double intDeltaTime = double(deltaTime);
  double error = remainderf((targetAngle - GyroValue),360);
  iError += error * intDeltaTime;
  double dError = (error - lastError) / intDeltaTime;

  double output = straightP * error + straightI * iError + dError * straightD;
  output = std::clamp(output, -0.7, 0.7);
  lastError = error;

  if (x2!=0) {
    targetAngle = GyroValue;
    ROT = x2;
  } else {
      ROT = output;
  }

  // Calculate variables A, B, C, D
  double A = (STR - ROT * L / 2);
  double B = (STR + ROT * L / 2);
  double C = (FWD - ROT * W / 2);
  double D = (FWD + ROT * W / 2);

  // Calculate wheel speeds
  if (x!=0||y!=0||x2!=0) {
  speedFL = (std::sqrt((B * B) + (D * D))+(triggerL+triggerR)); 
  speedFR = (std::sqrt((B * B) + (C * C))+(triggerL+triggerR)); 
  speedBL = (std::sqrt((A * A) + (D * D))+(triggerL+triggerR)); 
  speedBR = (std::sqrt((A * A) + (C * C))+(triggerL+triggerR));
  } else {
    speedFL = 0;
    speedFR = 0;
    speedBL = 0;
    speedBR = 0;
  }
  

  std::vector<double> wheelSpeeds = {speedFL, speedFR, speedBL, speedBR};

  // Find the maximum value among the wheel speeds
  double maxSpeed = *std::max_element(wheelSpeeds.begin(), wheelSpeeds.end());

  // If over one, divide all wheel speeds by the max speed
  if (maxSpeed >= 1) {
    speedFL = speedFL / maxSpeed;
    speedFR = speedFR / maxSpeed;
    speedBL = speedBL / maxSpeed;
    speedBR = speedBR / maxSpeed;
  }

  // Calculate final angle for each wheel
  if (x!=0||y!=0||x2!=0){
  tempangleFL = (std::atan2(B, D)+M_PI);
  angleFL = tempangleFL;
  fmod(angleFL + 2 * M_PI, 2 * M_PI);

  tempangleFR = (std::atan2(B, C)+M_PI);
  angleFR = tempangleFR;
  fmod(angleFR + 2 * M_PI, 2 * M_PI);

  tempangleBL = (std::atan2(A, D)+M_PI);
  angleBL = tempangleBL;
  fmod(angleBL + 2 * M_PI, 2 * M_PI);

  tempangleBR = (std::atan2(A, C)+M_PI);
  angleBR = tempangleBR;
  fmod(angleBR + 2 * M_PI, 2 * M_PI);
  }

  double errorFL = angleFL - FL_pos;
  double errorFR = angleFR - FR_pos;
  double errorBL = angleBL - BL_pos;
  double errorBR = angleBR - BR_pos;

  speedFL *= std::pow(cos(errorFL * M_PI/180),3);
  speedFR *= std::pow(cos(errorFR * M_PI/180),3);
  speedBL *= std::pow(cos(errorBL * M_PI/180),3);
  speedBR *= std::pow(cos(errorBR * M_PI/180),3);

  // Control steering motors to angle with PID

  //PID calculations
//FL
if (abs(errorFL) > M_PI / 2) {
    errorFL += (errorFL > 0 ? -M_PI : M_PI);
    speedFL = -speedFL;
}
if (abs(errorFL) > M_PI / 2) {
    errorFL += (errorFL > 0 ? -M_PI : M_PI);
    speedFL = -speedFL;
}

//FR
if (abs(errorFR) > M_PI / 2) {
    errorFR += (errorFR > 0 ? -M_PI : M_PI);
    speedFR = -speedFR;
}
if (abs(errorFR) > M_PI / 2) {
    errorFR += (errorFR > 0 ? -M_PI : M_PI);
    speedFR = -speedFR;
}

//BL
if (abs(errorBL) > M_PI / 2) {
    errorBL += (errorBL > 0 ? -M_PI : M_PI);
    speedBL = -speedBL;
}
if (abs(errorBL) > M_PI / 2) {
    errorBL += (errorBL > 0 ? -M_PI : M_PI);
    speedBL = -speedBL;
}

//BR
if (abs(errorBR) > M_PI / 2) {
    errorBR += (errorBR > 0 ? -M_PI : M_PI);
    speedBR = -speedBR;
}
if (abs(errorBR) > M_PI / 2) {
    errorBR += (errorBR > 0 ? -M_PI : M_PI);
    speedBR = -speedBR;
}
  //Front Left PID calculations
  double iErrorFL;
  iErrorFL += errorFL * doubleDeltaTime;
  double dErrorFL = (errorFL - lasterrorFL) / doubleDeltaTime;
  double steerm_speedFL = ModuleP * errorFL + ModuleI * iErrorFL + ModuleD * dErrorFL;
  lasterrorFL = errorFL;

  //Front Right PID calculations
  double iErrorFR;
  iErrorFR += errorFR * doubleDeltaTime;
  double dErrorFR = (errorFR - lasterrorFR) / doubleDeltaTime;
  double steerm_speedFR = ModuleP * errorFR + ModuleI * iErrorFR + ModuleD * dErrorFR;
  lasterrorFR = errorFR;
  frc::SmartDashboard::PutNumber("errorBL",errorBL);

  //Back Left PID calculations
  double iErrorBL;
  iErrorBL += errorBL * doubleDeltaTime;
  double dErrorBL = (errorBL - lasterrorBL) / doubleDeltaTime;
  double steerm_speedBL = ModuleP * errorBL + ModuleI * iErrorBL + ModuleD * dErrorBL;
  lasterrorBL = errorBL;

  //Back Right PID calculations
  double iErrorBR;
  iErrorBR += errorBR * doubleDeltaTime;
  double dErrorBR = (errorBR - lasterrorBR) / doubleDeltaTime;
  double steerm_speedBR = ModuleP * errorBR + ModuleI * iErrorBR + ModuleD * dErrorBR;
  lasterrorBR = errorBR;

  if (steerm_speedBR > 0.03||steerm_speedBR < -0.03) {
    m_FL_Steer.Set(std::clamp(steerm_speedFL, -0.25, 0.25));
    m_FR_Steer.Set(std::clamp(steerm_speedFR, -0.25, 0.25));
    m_BL_Steer.Set(std::clamp(steerm_speedBL, -0.25, 0.25));
    m_BR_Steer.Set(std::clamp(steerm_speedBR, -0.25, 0.25));
  } else {
    m_FL_Steer.Set(0);
    m_FR_Steer.Set(0);
    m_BL_Steer.Set(0);
    m_BR_Steer.Set(0);
  }
  //P gain = 0.200000
  //I gain = 0.010083
  //D gain = 0.000221

  double speedFLEr = speedFL - oldspeedFL;
  oldspeedFL += (speedFLEr >= 0.1) ? 0.1 : (speedFLEr <= -0.07) ? -0.07 : speedFLEr;
  double speedFREr = speedFR - oldspeedFR;
  oldspeedFR += (speedFREr >= 0.1) ? 0.1 : (speedFREr <= -0.07) ? -0.07 : speedFREr;
  double speedBLEr = speedBL - oldspeedBL;
  oldspeedBL += (speedBLEr >= 0.1) ? 0.1 : (speedBLEr <= -0.07) ? -0.07 : speedBLEr;
  double speedBREr = speedBR - oldspeedBR;
  oldspeedBR += (speedBREr >= 0.1) ? 0.1 : (speedBREr <= -0.07) ? -0.07 : speedBREr;

  //Control steering motors to velocity with PID
if (speedFL != speedFL){
  speedFL = 0;
  speedFR = 0;
  speedBL = 0;
  speedBR = 0;
} else {
  m_FL_Drive.Set(std::clamp(-oldspeedFL/1.25,-0.2,0.2)); 

  m_FR_Drive.Set(std::clamp(-oldspeedFR/1.25,-0.2,0.2));

  m_BL_Drive.Set(std::clamp(-oldspeedBL/1.25,-0.2,0.2)); 

  m_BR_Drive.Set(std::clamp(-oldspeedBR/1.25,-0.2,0.2));
}

/*
RotPerMin of DriveMotor/60 = RotPerSec of DriveMotor
Ans / 6.13(drivemotor to wheel gear ratio) = RotPerSec of Wheel
Ans * ~0.314(Circumference of wheel) = MetersPerSec
*/
double wheelSpeedFL = ((double(m_FL_Drive.GetVelocity().GetValue())/60)/6.13)*(M_PI/10);
double wheelSpeedFR = ((double(m_FR_Drive.GetVelocity().GetValue())/60)/6.13)*(M_PI/10);
double wheelSpeedBL = ((double(m_BL_Drive.GetVelocity().GetValue())/60)/6.13)*(M_PI/10);
double wheelSpeedBR = ((double(m_BR_Drive.GetVelocity().GetValue())/60)/6.13)*(M_PI/10);

  odometryUpdate(        
    FL_pos, 
    FR_pos, 
    BL_pos, 
    BR_pos, 
    wheelSpeedFL,
    wheelSpeedFR,
    wheelSpeedBL,
    wheelSpeedBR,
    GyroValue);
}

void Drivetrain::odometryUpdate(        
    double angleFL, 
    double angleFR, 
    double angleBL, 
    double angleBR, 
    double wheelSpeedFL,
    double wheelSpeedFR,
    double wheelSpeedBL,
    double wheelSpeedBR,
    double GyroValue) {

    units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
    double odoDeltaTime = double(odoCurrentTime) - double(odoLastTime);
    odoLastTime = odoCurrentTime; 

    double B_FL = sin(angleFL) * wheelSpeedFL;
    double B_FR = sin(angleFR) * wheelSpeedFR;
    double A_BL = sin(angleBL) * wheelSpeedBL;
    double A_BR = sin(angleBR) * wheelSpeedBR;

    double D_FL = cos(angleFL) * wheelSpeedFL;
    double C_FR = cos(angleFR) * wheelSpeedFR;
    double D_BL = cos(angleBL) * wheelSpeedBL;
    double C_BR = cos(angleBR) * wheelSpeedBR;

    double A = (A_BL + A_BR) / 2;
    double B = (B_FL + B_FR) / 2;
    double C = (C_FR + C_BR) / 2;
    double D = (D_FL + D_BL) / 2;

    odoROT = (GyroValue*M_PI/180)/double(odoCurrentTime);

    double FWD1 = ROT * (L / 2) + A;
    double FWD2 = -ROT * (L / 2) + B;
    odoFWD = (FWD1 + FWD2) / 2;

    double STR1 = ROT * (W / 2) + C;
    double STR2 = -ROT * (W / 2) + D;
    odoSTR = (STR1 + STR2) / 2;
    
    double temp = odoFWD * cos(GyroValue) + odoSTR * sin(GyroValue);
    odoSTR = odoSTR * cos(GyroValue) - odoFWD * sin(GyroValue);
    odoFWD = temp;

    positionFWDField -=  units::length::meter_t(odoFWD * odoDeltaTime);
    positionSTRField +=  units::length::meter_t(odoSTR * odoDeltaTime);
    ROTField = frc::Rotation2d(units::angle::radian_t(ROT));

    frc::SmartDashboard::PutNumber("positionFWDField", double(positionFWDField));
    frc::SmartDashboard::PutNumber("positionSTRField", double(positionSTRField));
}