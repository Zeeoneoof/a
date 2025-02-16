#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogInput.h>
#include <frc/Timer.h>
#include <frc2/command/PIDCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Orchestra.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Compressor.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include "math.h"

static const int numEncoders = 4;

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain : public frc2::SubsystemBase {
    public:
        void Update(double x, double y, double x2, double GyroValue, double triggerL, double triggerR,bool FieldCentric);

        void odometryUpdate(
        double angleFL, 
        double angleFR, 
        double angleBL, 
        double angleBR, 
        double wheelSpeedFL,
        double wheelSpeedFR,
        double wheelSpeedBL,
        double wheelSpeedBR,
        double GyroValue);

        Drivetrain();

        units::length::meter_t positionFWDField = units::length::meter_t(0);
        units::length::meter_t positionSTRField = units::length::meter_t(0);
        frc::Rotation2d ROTField = frc::Rotation2d(units::radian_t(0));
    private:
        //Kraken Drive motors *0
        ctre::phoenix6::hardware::TalonFX m_FL_Drive{30};//3
        ctre::phoenix6::hardware::TalonFX m_FR_Drive{20};//2
        ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};//4
        ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};//1

        //Kraken Steering motors *1
        ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};//3
        ctre::phoenix6::hardware::TalonFX m_FR_Steer{21};//2
        ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};//4
        ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};//1
        
        //kraken music player
        ctre::phoenix6::Orchestra music{};

        //setup CANcoder *2
        ctre::phoenix6::hardware::CANcoder CANcoderFL{32};//3
        ctre::phoenix6::hardware::CANcoder CANcoderFR{22};//2
        ctre::phoenix6::hardware::CANcoder CANcoderBL{42};//4
        ctre::phoenix6::hardware::CANcoder CANcoderBR{12};//1

        ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};   
        ctre::phoenix6::configs::TalonFXConfiguration configFL{};
        ctre::phoenix6::configs::TalonFXConfiguration configFR{};
        ctre::phoenix6::configs::TalonFXConfiguration configBL{};
        ctre::phoenix6::configs::TalonFXConfiguration configBR{};
        ctre::phoenix6::configs::CANcoderConfiguration cc_cfg{};

        //P gain = 0.200000
        //I gain = 0.010083
        //D gain = 0.000221
        float ModuleP = 0.2;
        float ModuleI = 0.0;
        float ModuleD = 0.0;
        float straightP = 0;
        float straightI = 0;
        float straightD = 0;

        // Constants for the robot dimensions and encoder configuration
        const float L = 0.5461;
        const float W = 0.5461;

        double tempangleFL, tempangleFR, tempangleBL, tempangleBR;
        double angleFL, angleFR, angleBL, angleBR;
        double speedFL, speedFR, speedBL, speedBR;

        double oldspeedFL, oldspeedFR, oldspeedBL, oldspeedBR = 0;

        units::time::second_t lastTime = units::time::second_t(0);
        units::time::second_t odoLastTime = units::time::second_t(0);
        double lasterrorFL, lasterrorFR, lasterrorBL, lasterrorBR = 0;

        double ROT = 0;
        double FWD = 0;
        double STR = 0;

        double targetAngle = 0;
        double ROT2 = 0;

        double wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR = 0;

        double odoROT = 0;
        double odoFWD = 0;
        double odoSTR = 0;

        double previousDegreesFL;
        double previousDegreesFR;
        double previousDegreesBL;
        double previousDegreesBR;

        double previousAngleFL;
        double previousAngleFR;
        double previousAngleBL;
        double previousAngleBR;
};
#endif