#pragma once

#include <rev/SparkMax.h>
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
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Compressor.h>
#include "math.h"

#include "Odometry.h"

static const int numEncoders = 4;

class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();
        void Update(double x, double y, double x2, double GyroValue, double triggerL, double triggerR,bool FieldCentric);
        void Drive(const frc::ChassisSpeeds& speeds);  // Simple placeholder method
    private:

        float ModuleP = 0.200000;
        float ModuleI = 0.010083;
        float ModuleD = 0.000221;
        float straightP = 0;
        float straightI = 0;
        float straightD = 0;
        
        //Talon FX
        //ctre::phoenix6::hardware::TalonFX m_FL_Drive2{11/*CAN ID*/};
        //ctre::phoenix6::hardware::CANcoder CANcoderFL{9/*CAN ID*/};

        //Drive motors
        rev::spark::SparkMax m_FL_Drive{11, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_FR_Drive{6, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_BL_Drive{13, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_BR_Drive{8, rev::spark::SparkLowLevel::MotorType::kBrushless};

        rev::spark::SparkRelativeEncoder FL_Dr_Encoder = m_FL_Drive.GetEncoder();
        rev::spark::SparkRelativeEncoder FR_Dr_Encoder = m_FR_Drive.GetEncoder();
        rev::spark::SparkRelativeEncoder BL_Dr_Encoder = m_BL_Drive.GetEncoder();
        rev::spark::SparkRelativeEncoder BR_Dr_Encoder = m_BR_Drive.GetEncoder();

        //Steering motors
        rev::spark::SparkMax m_FL_Steer{12, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_FR_Steer{5, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_BL_Steer{10, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_BR_Steer{7, rev::spark::SparkLowLevel::MotorType::kBrushless};

        frc::AnalogInput LampFL{0};
        frc::AnalogInput LampFR{3};
        frc::AnalogInput LampBL{2};
        frc::AnalogInput LampBR{1};

        // Constants for the robot dimensions and encoder configuration
        const float L = 0.5461;
        const float W = 0.5461;

        double tempangleFL, tempangleFR, tempangleBL, tempangleBR;
        double angleFL, angleFR, angleBL, angleBR;
        double speedFL, speedFR, speedBL, speedBR;

        double oldspeedFL, oldspeedFR, oldspeedBL, oldspeedBR = 0;

        units::time::second_t lastTime = units::time::second_t(0);
        double lasterrorFL, lasterrorFR, lasterrorBL, lasterrorBR = 0;

        double targetAngle = 0;
        double ROT2 = 0;
        double ROT = 0;

        double wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR = 0;

};