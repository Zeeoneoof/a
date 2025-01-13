#pragma once

#include <frc/Timer.h>
#include "math.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>

#include "Drivetrain.h"
class Drivetrain;

class Odometry {
    public:
        frc::Pose2d getPose();
        void ResetPose(const frc::Pose2d& pose);
        frc::ChassisSpeeds GetRobotRelativeSpeeds();
        void DriveRobotRelative(const frc::ChassisSpeeds& speeds);
        Odometry();
        void Update(
        double angleFL, 
        double angleFR, 
        double angleBL, 
        double angleBR, 
        double wheelSpeedFL,
        double wheelSpeedFR,
        double wheelSpeedBL,
        double wheelSpeedBR,
        double GyroValue); 
    private:
        const float L = 0.5461;
        const float W = 0.5461;

        units::time::second_t lastTime = units::time::second_t(0);
        std::unique_ptr<Drivetrain> drivetrain;
};