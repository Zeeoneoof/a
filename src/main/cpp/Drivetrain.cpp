#include <frc/smartdashboard/SmartDashboard.h>

#include "Drivetrain.h"

double currentDegrees[4];
double previousDegreesFL;
double previousDegreesFR;
double previousDegreesBL;
double previousDegreesBR;

double previousAngleFL;
double previousAngleFR;
double previousAngleBL;
double previousAngleBR;

double MaxFLV = 0;
double MaxFRV = 0;
double MaxBLV = 0;
double MaxBRV = 0;

double calculateAngle(double currentVoltage,int i,double MaxV){
  if (i < numEncoders){
    return currentDegrees[i] = ((currentVoltage/MaxV)*(2*M_PI));
  }
}

Drivetrain::Drivetrain () {
    rev::spark::SparkBaseConfig config;
    config.ClosedLoopRampRate(1);

    m_FL_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_FR_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BL_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BR_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    //Talon FX
    //m_FL_Drive2.ClearStickyFaults();
    //m_FL_Drive2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    
    rev::spark::SparkBaseConfig config2;

    m_FL_Drive.Configure(config2, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_FR_Drive.Configure(config2, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BL_Drive.Configure(config2, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BR_Drive.Configure(config2, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    m_FL_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_FR_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BL_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BR_Drive.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    m_FL_Steer.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_FR_Steer.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BL_Steer.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_BR_Steer.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}
Odometry m_odometry;
void Drivetrain::Update (double x, double y, double x2, double GyroValue, double triggerL, double triggerR, bool FieldCentric)  {

  straightP = frc::SmartDashboard::GetNumber("Straight P", 0.02);
  straightI = frc::SmartDashboard::GetNumber("Straight I", 0);
  straightD = frc::SmartDashboard::GetNumber("Not Straight D", 0);

  //ctre::phoenix6::StatusSignal<units::angle::turn_t> CANcoderFLAngle = CANcoderFL.GetPosition();
  //double CANcoderFLpos = double(CANcoderFLpos);

  // Code executed periodically during robot operation
    // Retrieve voltage values from encoders
    double LampFLV = LampFL.GetVoltage();
    double LampFRV = LampFR.GetVoltage();
    double LampBLV = LampBL.GetVoltage();
    double LampBRV = LampBR.GetVoltage();

    double currentVoltage[4] = {LampFLV,LampFRV,LampBLV,LampBRV};
    
    float calibSpeed = 0.2;
    if (LampFLV > MaxFLV) {
      MaxFLV = LampFLV;
    }

    if (LampFRV > MaxFRV) {
      MaxFRV = LampFRV;
    }

        if (LampBLV > MaxBLV) {
      MaxBLV = LampBLV;
    }

        if (LampBRV > MaxBRV) {
      MaxBRV = LampBRV;
    }

    for (int i=0;i<numEncoders;i++) {
      double MaxV[4] = {MaxFLV,MaxFRV,MaxBLV,MaxBRV};
      calculateAngle(currentVoltage[i],i,MaxV[i]);
    }

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

  double errorFL = angleFL - currentDegrees[0];
  double errorFR = angleFR - currentDegrees[1];
  double errorBL = angleBL - currentDegrees[2];
  double errorBR = angleBR - currentDegrees[3];

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

  if (int(currentTime) <= 4) {
    m_FL_Steer.Set(calibSpeed);
    m_FR_Steer.Set(calibSpeed);
    m_BL_Steer.Set(calibSpeed);
    m_BR_Steer.Set(calibSpeed);
  } else {
    if (steerm_speedBR > 0.01||steerm_speedBR < -0.01) {
      m_FL_Steer.Set(std::clamp(-steerm_speedFL, -0.6, 0.6));
      m_FR_Steer.Set(std::clamp(-steerm_speedFR, -0.6, 0.6));
      m_BL_Steer.Set(std::clamp(-steerm_speedBL, -0.6, 0.6));
      m_BR_Steer.Set(std::clamp(-steerm_speedBR, -0.6, 0.6));
    } else {
      m_FL_Steer.Set(0);
      m_FR_Steer.Set(0);
      m_BL_Steer.Set(0);
      m_BR_Steer.Set(0);
    }
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
  m_FL_Drive.Set(std::clamp(oldspeedFL/1.25,-0.8,0.8)); 

  m_FR_Drive.Set(std::clamp(oldspeedFR/1.25,-0.8,0.8));

  m_BL_Drive.Set(std::clamp(oldspeedBL/1.25,-0.8,0.8)); 

  m_BR_Drive.Set(std::clamp(oldspeedBR/1.25,-0.8,0.8));
}

/*
RotPerMin of DriveMotor/60 = RotPerSec of DriveMotor
Ans / 8.8(drivemotor to wheel gear ratio) = RotPerSec of Wheel
Ans * ~0.314(Circumference of wheel) = MetersPerSec
*/
double wheelSpeedFL = ((FL_Dr_Encoder.GetVelocity()/60)/8.8)*(M_PI/10);
double wheelSpeedFR = ((FR_Dr_Encoder.GetVelocity()/60)/8.8)*(M_PI/10);
double wheelSpeedBL = ((BL_Dr_Encoder.GetVelocity()/60)/8.8)*(M_PI/10);
double wheelSpeedBR = ((BR_Dr_Encoder.GetVelocity()/60)/8.8)*(M_PI/10);


m_odometry.Update(        
    currentDegrees[0], 
    currentDegrees[1], 
    currentDegrees[2], 
    currentDegrees[3], 
    wheelSpeedFL,
    wheelSpeedFR,
    wheelSpeedBL,
    wheelSpeedBR,
    GyroValue);
}