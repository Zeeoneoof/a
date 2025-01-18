/*#include "Drivetrain.h"
//#include "Robot.h"

    double ROT = 0;
    double FWD = 0;
    double STR = 0;
    units::length::meter_t positionFWDField = units::length::meter_t(0);
    units::length::meter_t positionSTRField = units::length::meter_t(0);

    //Drivetrain m_swerve;
Drivetrain& m_swerve = Drivetrain::getInstance();

void Drivetrain::Odometry () {

    //Drivetrain m_swerve;
    // Configure the AutoBuilder last
    pathplanner::RobotConfig robot_config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this]() {return getPose();},
        [this](frc::Pose2d pose) {ResetPose(pose);},
        [this]() { return GetRobotRelativeSpeeds(); },
        [this](auto speeds) {Drive(speeds);},
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
        pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    ),
        robot_config,
        []() { return true; },
        &Drivetrain::getInstance()
     );
}

frc2::CommandPtr Drivetrain::getAutonomousCommand(){
    // Load the path you want to follow using its name in the GUI
    auto path = pathplanner::PathPlannerPath::fromPathFile("Example Path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return pathplanner::AutoBuilder::followPath(path);
}

frc::Pose2d Drivetrain::getPose() {
    // Return the robot's current pose (e.g., from odometry).
    return frc::Pose2d(positionFWDField, positionSTRField, frc::Rotation2d(units::radian_t(ROT)));
}

void Drivetrain::ResetPose(const frc::Pose2d& pose) {
    // Reset the robot's odometry to the specified pose
    positionFWDField = units::length::meter_t(0);
    positionSTRField = units::length::meter_t(0);
    ROT = 0;
}

frc::ChassisSpeeds Drivetrain::GetRobotRelativeSpeeds() {
    // Return the current robot-relative ChassisSpeeds
    return frc::ChassisSpeeds(units::velocity::meters_per_second_t(FWD), units::velocity::meters_per_second_t(STR), units::angular_velocity::radians_per_second_t(ROT)); //(vx, vy, omega)
}

void Drivetrain::Drive(auto speeds) {
    m_swerve.Update(double(speeds.vx), double(speeds.vy), double(speeds.omega), 0, 0, 0, false);
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

    units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
    double deltaTime = double(currentTime) - double(lastTime);
    lastTime = currentTime; 

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

    ROT = (GyroValue*M_PI/180)/double(currentTime);

    double FWD1 = ROT * (L / 2) + A;
    double FWD2 = -ROT * (L / 2) + B;
    FWD = (FWD1 + FWD2) / 2;

    double STR1 = ROT * (W / 2) + C;
    double STR2 = -ROT * (W / 2) + D;
    STR = (STR1 + STR2) / 2;
    
    double temp = FWD * cos(GyroValue) + STR * sin(GyroValue);
    STR = STR * cos(GyroValue) - FWD * sin(GyroValue);
    FWD = temp;

    positionFWDField -=  units::length::meter_t(FWD * deltaTime);
    positionSTRField +=  units::length::meter_t(STR * deltaTime);
}*/