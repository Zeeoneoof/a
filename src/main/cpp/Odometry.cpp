#include "Odometry.h"
#include "Drivetrain.h"

Odometry::Odometry () {
    // Configure the AutoBuilder last
    struct RobotConfig {
        double maxSpeed;     // Maximum speed of the robot
        double maxAngularSpeed; // Maximum angular speed of the robot
    };
    Drivetrain drivetrain;
    RobotConfig robotConfig{3.0, 1.5};
    bool shouldFlipPath = false;
        pathplanner::AutoBuilder::configure(
            [this](units::length::meter_t positionFWDField, units::length::meter_t positionSTRField, units::degree_t ROT) {return getPose();},
            [this](const frc::Pose2d& pose) {ResetPose(pose);},
            [this]() -> frc::ChassisSpeeds { return frc::ChassisSpeeds{1_mps, 0_mps, 0.5_rad_per_s}; },
            [&drivetrain](const frc::ChassisSpeeds& speeds) {        // Lambda to drive the robot
                drivetrain.Drive(speeds);
            },
            controller,
            robotConfig,
            []() -> bool { return true; },
            &drivetrain
         );
}

frc2::CommandPtr GetAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    auto path = pathplanner::PathPlannerPath::fromPathFile("Example Path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return pathplanner::AutoBuilder::followPath(path);
}

frc::Pose2d getPose() {
    // Return the robot's current pose (e.g., from odometry).
    return frc::Pose2d(positionFWDField, positionSTRField, frc::Rotation2d(ROT));
}

void ResetPose(const frc::Pose2d& pose) {
    // Reset the robot's odometry to the specified pose
}

frc::ChassisSpeeds GetRobotRelativeSpeeds() {
    // Return the current robot-relative ChassisSpeeds
}

void DriveRobotRelative(const frc::ChassisSpeeds& speeds) {
    // Command the robot to move with the specified robot-relative ChassisSpeeds
}   

void Odometry::Update(        
    double angleFL, 
    double angleFR, 
    double angleBL, 
    double angleBR, 
    double wheelSpeedFL,
    double wheelSpeedFR,
    double wheelSpeedBL,
    double wheelSpeedBR,
    double GyroValue) {

    frc::SmartDashboard::PutNumber("wheelSpeedFL", wheelSpeedFL);
    frc::SmartDashboard::PutNumber("wheelSpeedFR", wheelSpeedFR);
    frc::SmartDashboard::PutNumber("wheelSpeedBL", wheelSpeedBL);
    frc::SmartDashboard::PutNumber("wheelSpeedBR", wheelSpeedBR);

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

    ROT = units::degree_t((GyroValue*M_PI/180)/double(currentTime));

    double FWD1 = ROT * (L / 2) + A;
    double FWD2 = -ROT * (L / 2) + B;
    double FWD = (FWD1 + FWD2) / 2;

    double STR1 = ROT * (W / 2) + C;
    double STR2 = -ROT * (W / 2) + D;
    double STR = (STR1 + STR2) / 2;
    
    double temp = FWD * cos(GyroValue) + STR * sin(GyroValue);
    STR = STR * cos(GyroValue) - FWD * sin(GyroValue);
    FWD = temp;

    positionFWDField -= FWD * deltaTime;
    positionSTRField += STR * deltaTime;

    //frc::SmartDashboard::PutNumber("positionFWDField", positionFWDField);
    //frc::SmartDashboard::PutNumber("positionSTRField", positionSTRField);
}