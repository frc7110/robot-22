// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/AnalogInput.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/I2C.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <cameraserver/CameraServer.h>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "pigeon_gyro.h"

static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;

typedef struct
{
    double y;
    double z;
    double t;
    double intake;
    int flywheel;
    double distance;
    double heading;
    bool pixy;
} move_step_t;

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

struct output_t
{
  output_t()
    : x(0.0), y(0.0), z(0.0), intake(0.0), lift(0.0), flywheel(0.0)
    {};

  double x;
  double y;
  double z;
  double intake;
  double lift;
  double flywheel;
};

class Robot : public frc::TimedRobot 
{
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

  void set_camera_selection();

  void start_move(move_step_t *steps, int count);
  void evaluate_step(output_t &output);
  void next_step(double t);

  double drive_distance(double d, bool absolute=false);
  double drive_heading(double h, bool absolute=false);
  bool steer_pixy(double &z, int color);

  void update_outputs(output_t output);
  void drive(double y, double z, bool limit_accel=false);

  int get_Pixy_xy(int& x, int& a, int& colorSignature);

 private:
    frc::Joystick m_stick_d {0};
    frc::Joystick m_stick_o {1};
    
    rev::CANSparkMax m_ll_motor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rl_motor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_lf_motor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rf_motor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_flywheel{5, rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_ll_encoder = m_ll_motor.GetEncoder();
    rev::SparkMaxRelativeEncoder m_rl_encoder = m_rl_motor.GetEncoder();

    frc::DifferentialDrive m_robotDrive{m_ll_motor, m_rl_motor};

    frc::Timer m_timer;

    frc::pigeon_gyro m_gyro {0};

    WPI_TalonSRX m_intake_1 {1};
    WPI_TalonSRX m_intake_2 {3};

    WPI_VictorSPX m_lift {2};

    frc::AnalogInput m_lift_position {0};

    cs::UsbCamera camera1;
    cs::UsbCamera camera2;
    // cs::VideoSink server;

    nt::NetworkTableEntry cameraSelection;

    frc2::PIDController m_lift_pid {0.1, 0, 0};

    frc::I2C m_pixyPort{frc::I2C::Port::kOnboard, 0x54};
  
    frc::DriverStation::Alliance m_alliance {frc::DriverStation::Alliance::kInvalid};

    int m_camera {1};

    double m_fw_sp1;
    double m_fw_sp2;
    int m_lift_sp {0};

    double m_direction {-1};
    double accel_y_max {0.005};
    double accel_z_max {0.01};

    size_t step_n;
    double m_step_start;

    bool move_active {0};

    move_step_t m_step;

    bool m_drive_active = false;
    bool m_heading_active = false;
};
