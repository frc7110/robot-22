// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

static const char *version = "Rapid React v5.10.1";

// speed limits
// -turbo mode
static double y_max_v_turbo = .8;
// - lift is down
static double y_max_v_dn = .6;
static double z_max_v_dn = .3;
// - lift is up
static double y_max_v_up = .32;
static double z_max_v_up = .25;

// acceleration limits
static double accel_y_max = 0.025;
static double accel_z_max = 0.02;

// lift setpoints
static double lift_high = 1340;
static double lift_low = 400;

static const int key_alt = 6;     // RB
static const int key_retract = 1; // A
static const int key_extend = 4;  // Y

// autonomous operations
// - all sequences should have a delay line

// table columns are:
// y speed, z speed, time, intake speed, flywheel speed selection, distance, heading, pixy enable

// shoot and back away
move_step_t mv_auto_1[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // back up
  {-0.2, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
};

// 2 cargo - straight back
move_step_t mv_auto_2[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 2.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // back up with intake running
  {-0.22, 0.0, 2.5, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, 0.0, 1, 0.0, 0.0, 0},
  // reverse
  {0.22, 0.0, 2.4, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
};

// 2 cargo - straight back
move_step_t mv_auto_2b[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // back up with intake running
  {-0.22, 0.0, 2.0, 0.0, 0, 0.0, 0.0, 0},
  {-0.22, 0.0, 0.7, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 0, 0.0, 0.0, 0},
  // reverse
  {0.22, 0.0, 2.4, 0.0, 1, 0.0, 0.0, 0},
  // fire 1
  {0, 0.0, 0.5, 0.7, 1, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 1.2, 0.0, 1, 0.0, 0.0, 0},
  // fire 2
  {0.0, 0.0, 0.5, 0.7, 1, 0.0, 0.0, 0},
};

// 2 cargo - straight back
move_step_t mv_auto_3[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // slight turn, back up with intake running
  {0.0, 0.0, 1.5, 0.7, 0, 0.0, -30, 0},
  {0.0, 0.0, 2.5, 0.7, 0, 80.0, 0.0, 1},
  {0.0, 0.0, 1.0, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // reverse
  {0.0, 0.0, 2.4, 0.0, 1, -70.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.0, 1, 0.0, 30, 0},
  // // fire
  {0.0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
};

// 2 cargo - pixy back
move_step_t mv_auto_4[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // slight turn, back up with intake running
  {0.0, 0.0, 1.5, 0.7, 0, 0.0, -25, 0},
  {-0.22, 0.0, 2.5, 0.7, 0, 0.0, 0.0, 1},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // reverse
  {0.22, 0.0, 2.4, 0.0, 1, 0.0, 0.0, 0},
  {0.0, 1.0, 1.0, 0.0, 1, 0.0, 1.0, 0},
  // fire
  {0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},

  // rotate toward terminal
  {0.0, 0.0, 5.5, 0.7, 0, 0.0, -40.0, 0},
  // drive toward terminal
  {0.0, 0.0, 5.5, 0.7, 0, -150, 0.0, 0},
  // enable pixy
  {-0.22, 0.0, 5.5, 0.7, 0, 0, 0.0, 1},
  // drive toward hub
  {0.0, 0.0, 5.5, 0.7, 0, 170, 0.0, 0},
  // rotate toward hub
  {0.0, 0.0, 5.5, 0.7, 0, 0.0, 40.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // fire
  {0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
};

// 2 cargo + terminal
move_step_t mv_auto_5[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // slight turn, back up with intake running
  {0.0, 0.0, 1.5, 0.7, 0, 0.0, -30, 0},
  {0.0, 0.0, 2.5, 0.7, 0, 80.0, 0.0, 1},
  {0.0, 0.0, 1.0, 0.7, 0, 0.0, 0.0, 0},

  {0.0, 0.0, 1.5, 0.7, 0, 0.0, -10, 0},
  {0.0, 0.0, 2.5, 0.7, 0, 105.0, 0.0, 1},
  {0.0, 0.0, 1.0, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // reverse
  {0.0, 0.0, 2.4, 0.0, 1, -200.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.0, 1, 0.0, 35, 0},
  // // fire
  {0.0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
};

// auton selections - position determines numbering
// - first entry is selection #1
// - all auton moves must be defined above this line
struct auto_tbl_t auton_selections[] =
{
  {AUTON(mv_auto_1)},
  {AUTON(mv_auto_2)},
  {AUTON(mv_auto_3)},
  {AUTON(mv_auto_4)},
  {AUTON(mv_auto_5)},
};

// additional computer assisted operations

// shoot
move_step_t mv_shoot_low[] =
{
  // all cargo to the top
  {0.0, 0.0, 0.7, 0.5, 0, 0.0, 0.0, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.3, -0.3, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 2, 0.0, 0.0, 0},
  // fire 1
  {0.0, 0.0, 0.3, 0.7, 2, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 0.5, 0.0, 2, 0.0, 0.0, 0},
  // fire 2
  {0.0, 0.0, 0.3, 0.7, 2, 0.0, 0.0, 0},
};

// back up
move_step_t mv_back[] =
{
  // back up
  {0.0, 0.0, 0.7, 0.0, 0, 12, 0.0, 0},
};

// back up and fire
move_step_t mv_back_shot[] =
{
  // back up
  {0.0, 0.0, 0.7, 0.0, 1, 16, 0.0, 0},
  // all cargo to the top
  // {0.0, 0.0, 0.5, 0.5, 0, 0.0, 0.0, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.6, -0.3, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire 1
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 0.9, 0.0, 1, 0.0, 0.0, 0},
  // fire 2
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
};

move_step_t mv_back_shot2[] =
{
  // back cargo away from flywheel
  // - while starting to spin it up?!
  {0.0, 0.0, 0.6, -0.3, 1, 0.0, 0.0, 0},
  // back up
  {0.0, 0.0, 0.7, 0.0, 1, 16, 0.0, 0},
  // additional spin up?
  {0.0, 0.0, 0.5, 0.0, 1, 0.0, 0.0, 0},
  // fire 1
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 0.9, 0.0, 1, 0.0, 0.0, 0},
  // fire 2
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
};

// "chamber" cargo
move_step_t mv_load[] =
{
  // all cargo to the top
  {0.0, 0.0, 0.5, 0.7, 0, 0.0, 0.0, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.5, -0.3, 0, 0.0, 0.0, 0},
};

// eject cargo
move_step_t mv_eject[] =
{
  // all cargo to the top
  {0.0, 0.0, 0.5, 0.5, 0, 0.0, 0.0, 0},
  // spin up to low speed
  {0.0, 0.0, 0.5, 0.0, 3, 0.0, 0.0, 0},
  // feed cargo up
  {0.0, 0.0, 0.5, 0.7, 3, 0.0, 0.0, 0},
  // speed up to eject
  {0.0, 0.0, 0.5, 0.0, 1, 0.0, 0.0, 0},
};

move_step_t mv_turn[] =
{
  // turn test
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 15.0, 0},
  {0.0, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, -45.0, 0},
  {0.0, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
  {0.0, 1.0, 2.0, 0.0, 0, 0.0, 1.0, 0},
};

move_step_t mv_drive[] =
{
  // drive_test
  {0.0, 0.0, 0.5, 0.0, 0, 6.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
  {0.0, 0.0, 0.5, 0.0, 0, -10.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
  {0.1, 0.0, 2.0, 0.0, 0, 0.1, 0.0, 0},
};

move_step_t mv_shoot_low_game[] =
{
  {0.0, 0.0, 0.2, -0.3, 0, 0, 0, 0},
  {0.0, 0.0, 2, 0.0, 3, 0, 0, 0},
  {0.0, 0.0, 2, 0.8, 3, 0, 0, 0},
};

move_step_t none[] =
{
	{0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
};

move_seq_t mv = {none, 0};


void clamp(double &value, const double ul, const double ll)
{
    if (value < ll)
  {
      value = ll;
  }
    else if (value > ul)
  {
      value = ul;
  }
}

void scale(double &value, const double deadband, const double ll, const double ul)
{
  double sign = (value >= 0.0) ? 1.0 : -1.0;

  value *= sign;  // force to positive

  //printf("0: value=%5.2f\n", value);

  value *= value;  // squaring value spreads out the center range

  //printf("1: value=%5.2f\n", value);

  // adjust for deadband
  if (value > deadband) value -= deadband;
  else value = 0;

  //printf("2: value=%5.2f\n", value);

  // scale based on output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

  //printf("3: value=%5.2f\n", value);

  // offset to minimum
  value += ll;

  value *= sign;  // restore sign

  //printf("4: value=%5.2f\n", value);
}

double normalize(double v)
{
  while (v < -1.0 || v > 1.0)  v /= 10;

  return v;
}

#if 0
//Attempt to set up advanced camera server program. Will need more work. 
void VisionThread() 
{  
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  camera.SetResolution(320, 240);
  camera.SetFPS(15);

  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStreamStd = frc::CameraServer::PutVideo("Gray", 320, 240);
  cv::Mat source;
  cv::Mat output;
  while(true) {
    if (cvSink.GrabFrame(source) == 0) {
      continue;
    }

    cv::line(source, cv::Point(160, 0), cv::Point(160, 230), cv::Scalar(0, 255, 0), 2, cv::LineTypes::LINE_8, 0);

    outputStreamStd.PutFrame(source);
  }
}
#endif

void Robot::RobotInit() 
{
    printf("%s%s %s %s\n", version, BRANCH, __DATE__, __TIME__);

    m_ll_motor.RestoreFactoryDefaults();
    m_rl_motor.RestoreFactoryDefaults();
    m_lf_motor.RestoreFactoryDefaults();
    m_rf_motor.RestoreFactoryDefaults();

    m_lf_motor.Follow(m_ll_motor);
    m_rf_motor.Follow(m_rl_motor);

    m_ll_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_lf_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rf_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_rl_motor.SetInverted(true);

    // have encoder report position in inches instead of revs
    m_ll_encoder.SetPositionConversionFactor(1.6);
    m_rl_encoder.SetPositionConversionFactor(1.6);

    m_lift.SetNeutralMode(NeutralMode::Brake);
    
    m_timer.Start();
    
    // set reference postion and orientation
    m_ll_encoder.SetPosition(0);
    m_rl_encoder.SetPosition(0);

#ifdef VELOCITY_CONTROL    
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
#endif

    m_gyro.Reset();

#if 1
    camera1 = frc::CameraServer::StartAutomaticCapture();
    camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera1.SetResolution(320, 240);
    // camera1.SetResolution(320, 180);  // Lifecam supports 16:9?
    camera1.SetFPS(15);
#endif
#if 0
    camera2 = frc::CameraServer::StartAutomaticCapture();
    camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera2.SetResolution(320, 240);
    // camera2.SetResolution(320, 180);  // Lifecam supports 16:9?
    camera2.SetFPS(15);
#endif

//    std::thread visionThread(VisionThread);
//    visionThread.detach();

    cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");

    // sync direction and camera selection
    set_camera_selection();

    frc::SmartDashboard::PutNumber("delay", 0);
    frc::SmartDashboard::PutNumber("auto", 2);

    frc::SmartDashboard::PutNumber("fw_sp1", 0.56);
    frc::SmartDashboard::PutNumber("fw_sp2", 0.4);
    frc::SmartDashboard::PutNumber("fw_sp3", 0.35);

    frc::SmartDashboard::PutNumber("P", 0.002);
    frc::SmartDashboard::PutNumber("I", 0.000001);

    frc::SmartDashboard::PutBoolean("TURBO", false);

    m_alliance = frc::DriverStation::GetAlliance();

    m_timer.Start();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
    double delay = frc::SmartDashboard::GetNumber("delay", 0);
    unsigned int auto_selection = frc::SmartDashboard::GetNumber("auto", 1);

    m_fw_sp1 = normalize(frc::SmartDashboard::GetNumber("fw_sp1", 0));
    m_fw_sp2 = normalize(frc::SmartDashboard::GetNumber("fw_sp2", 0));
    m_fw_sp3 = normalize(frc::SmartDashboard::GetNumber("fw_sp3", 0));

    printf("fw_speed=%5.2f/%5.2f/%5.2f\n", m_fw_sp1, m_fw_sp2, m_fw_sp3);

    // set reference postion and orientation
    m_ll_encoder.SetPosition(0);
    m_rl_encoder.SetPosition(0);

    m_gyro.Reset();

    m_timer.Start();

    printf("auto=%d, delay=%5.2f\n", auto_selection, delay);

    auto_selection -= 1;  // zero based indexing
    if (auto_selection < SIZEOF_ARRAY(auton_selections))
    {
      start_move(auton_selections[auto_selection].move, auton_selections[auto_selection].n);
      m_step.t = delay;
    }
    else
    {
      start_move(none, 1);
    }
}

void Robot::AutonomousPeriodic() 
{
    output_t output;
    evaluate_step(output);

    update_outputs(output);
}

void Robot::TeleopInit() 
{
    // reading the dashboard here simplifies test cycles
    m_fw_sp1 = normalize(frc::SmartDashboard::GetNumber("fw_sp1", 0));
    m_fw_sp2 = normalize(frc::SmartDashboard::GetNumber("fw_sp2", 0));
    m_fw_sp3 = normalize(frc::SmartDashboard::GetNumber("fw_sp3", 0));

    printf("fw_speed=%5.2f/%5.2f/%5.2f\n", m_fw_sp1, m_fw_sp2, m_fw_sp3);

    turbo = frc::SmartDashboard::GetBoolean("TURBO", false);

    m_alliance = frc::DriverStation::GetAlliance();

    double P = frc::SmartDashboard::GetNumber("P", 0);
    // double I = frc::SmartDashboard::GetNumber("I", 0);

    m_lift_pid.SetP(P);
    // m_lift_pid.SetI(I);

    m_lift_sp = m_lift_position.GetValue();

    move_active = false;

    // set reference postion and orientation
    m_ll_encoder.SetPosition(0);
    m_rl_encoder.SetPosition(0);

    m_gyro.Reset();
}

void Robot::TeleopPeriodic() 
{
    output_t output;

#if 0
    // select camera
    if (m_stick_d.GetRawButtonPressed(5))
    {
      m_camera = (m_camera == 1) ? 2 : 1;
      set_camera_selection();
    }
#endif

    // driver input
    double y = m_stick_d.GetRawAxis(1) * m_direction;
    double z = m_stick_d.GetRawAxis(4);

if (turbo) {

    if (m_lift_sp < 1.20 * lift_low)
    {
      if (m_stick_d.GetRawButton(5))
      {
        // turbo is mode only allowed with lift down
        scale(y, 0.05, 0.0, 1);
      }
      else
      {
        scale(y, 0.05, 0.0, y_max_v_dn);
      }
      scale(z, 0.05, 0.0, z_max_v_dn);
    }
    else
    {
      scale(y, 0.05, 0.0, y_max_v_up);
      scale(z, 0.05, 0.0, z_max_v_up);
    }
  
    output.y = y;
    output.z = z;

    double lift_speed = 0;
    double fw_speed = 0;
    double intake_speed = 0;

    //flywheel control
    if (m_stick_d.GetRawButton(6))
    {
      fw_speed = m_fw_sp1;
    }
    else if (m_stick_d.GetRawAxis(3) > 0.5)
    {
      fw_speed = m_fw_sp2;
    }
    output.flywheel = fw_speed;

    // intake control
    if (m_stick_o.GetRawButton(2))
    {
      intake_speed = 0.8;
    }
    else if (m_stick_o.GetRawButton(3))
    {
      intake_speed = -0.3;
    }
    output.intake = intake_speed;

    // lift control
    int lift_position = m_lift_position.GetValue();   

    if (!m_stick_o.GetRawButton(key_alt))
    {
      if (m_stick_o.GetRawButtonPressed(key_extend))
      {
        m_lift_sp = lift_high;
      }
      else if (m_stick_o.GetRawButtonPressed(key_retract))
      {
        m_lift_sp = lift_low;
      }

      lift_speed = m_lift_pid.Calculate(lift_position, m_lift_sp);
    }
    else
    {
      // manual control - no limits in case of pot failure!
      if (m_stick_o.GetRawButton(key_extend))
      {
        m_lift_sp = lift_position;

        // if (lift_position < m_lift_sp) lift_speed = 0.4;
        lift_speed = 0.4; 
      }
      else if (m_stick_o.GetRawButton(key_retract))
      {
        m_lift_sp = lift_position;

        // if (lift_position > m_lift_sp) lift_speed = -0.4;
        lift_speed = -0.4; 
      }
    }
    clamp(lift_speed, 0.7, -0.7);
    // scale(lift_speed, 0.1, 0.3, 0.7);
    // printf("sp=%d pos=%d lift speed = %5.2f\n", m_lift_sp, lift_position, lift_speed);
    output.lift = lift_speed;

    // start auto move?
    if (m_stick_d.GetRawButtonPressed(2))
    {
      START_MOVE(mv_shoot_low);
    }
    else if (m_stick_d.GetRawButtonPressed(3))
    {
      START_MOVE(mv_back_shot);
    }

    evaluate_step(output);

    if (m_stick_d.GetRawAxis(2) > 0.5)
    {
        int color = m_alliance;
        if (m_stick_d.GetRawButton(4)) color = (m_alliance == 1 ? 2 : 1);

        // get z from the pixy
        steer_pixy(output.z, color);
    }

#if 0
    if (output.flywheel != 0) printf("fw v=%5.2f\n", m_fw_encoder.GetVelocity());
    if (output.flywheel != 0) frc::SmartDashboard::PutNumber("fw", m_fw_encoder.GetVelocity());
#endif

    // all motors should be updated every pass
    update_outputs(output);
}
#if 0
else {
  scale(y, 0.05, 0, 0.3);
  scale(z, 0.05, 0, 0.25);

  double fw_speed = 0;

if (m_stick_d.GetRawAxis(3) > 0.5)
    {
      fw_speed = m_fw_sp2;
    }
    output.flywheel = fw_speed;

  m_intake_2.Set(0.8);

if (m_stick_o.GetRawButton(2))
    {
      m_intake_1.Set(0.8);
    }
    else if (m_stick_o.GetRawButton(3))
    {
      m_intake_1.Set(-0.3);
    } else{ 
      m_intake_1.Set(0);
    }

    double lift_speed = 0;
clamp(lift_speed, -0.7, 0.7);
int lift_position = m_lift_position.GetValue();   
lift_speed = m_lift_pid.Calculate(lift_position, m_lift_sp);

      if (m_stick_o.GetRawButtonPressed(key_extend))
      {
        m_lift_sp = lift_high;
      }
      else if (m_stick_o.GetRawButtonPressed(key_retract))
      {
        m_lift_sp = lift_low;
      }

      lift_speed = m_lift_pid.Calculate(lift_position, m_lift_sp);
          double lift_speed = 0;
    }
}
#endif
else{
  
    //drive controls
    #if 0
    if (m_lift_sp < 1.20 * lift_low)
    {
      if (m_stick_d.GetRawButton(5))
      {
        // turbo is mode only allowed with lift down
        scale(y, 0.05, 0.0, y_max_v_turbo);
      }
      else
      {
        scale(y, 0.05, 0.0, y_max_v_dn);
      }
      scale(z, 0.05, 0.0, z_max_v_dn);
    }
    else
    {
      scale(y, 0.05, 0.0, y_max_v_up);
      scale(z, 0.05, 0.0, z_max_v_up);
    }
    #endif
    if(m_lift_sp < 1.20 * lift_low){
      scale(y, 0.05, 0.0, 0.3);
      scale(z, 0.05, 0.0, 0.25);
    } else {
    scale(y, 0.05, 0.0, 0.15);
    scale(z, 0.05, 0.0, 0.125);
    }
    output.y = y;
    output.z = z;

    double lift_speed = 0;
    double fw_speed = 0;
    double intake_speed = 0;

    //flywheel control
    if (m_stick_d.GetRawAxis(3) > 0.5)
    {
      fw_speed = 0.35; //default is m_fw_sp2
    }
    output.flywheel = fw_speed;

    // intake control
    if (m_stick_d.GetRawButton(5))
    {
      intake_speed = 0.8;
    }
    else if (m_stick_d.GetRawAxis(2) > 0.5)
    {
      intake_speed = -0.3;
    } 
    else {
      m_intake_2.Set(0);
    }
    output.intake = intake_speed;

    // lift control
    int lift_position = m_lift_position.GetValue();   

    if (!m_stick_d.GetRawButton(key_alt))
    {
      if (m_stick_d.GetRawButtonPressed(key_extend))
      {
        m_lift_sp = 1600; //1340 is standard
      }
      else if (m_stick_d.GetRawButtonPressed(key_retract))
      {
        m_lift_sp = 400; //400 is standard
      }

      lift_speed = m_lift_pid.Calculate(lift_position, m_lift_sp);
    }
    else
    {
      // manual control - no limits in case of pot failure!
      if (m_stick_d.GetRawButton(key_extend))
      {
        m_lift_sp = lift_position;

        // if (lift_position < m_lift_sp) lift_speed = 0.4;
        lift_speed = 0.4; 
      }
      else if (m_stick_d.GetRawButton(key_retract))
      {
        m_lift_sp = lift_position;

        // if (lift_position > m_lift_sp) lift_speed = -0.4;
        lift_speed = -0.4; 
      }
    }
    clamp(lift_speed, 0.7, -0.7);
    // scale(lift_speed, 0.1, 0.3, 0.7);
    // printf("sp=%d pos=%d lift speed = %5.2f\n", m_lift_sp, lift_position, lift_speed);
    output.lift = lift_speed;

    if(m_stick_d.GetRawButton(6)) {
      START_MOVE(mv_shoot_low_game);
    }

    evaluate_step(output);

    update_outputs(output);
}
}

void Robot::DisabledInit() 
{
    m_ll_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_lf_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rf_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::set_camera_selection()
{
  if (m_camera == 1)
  {
    printf("set camera 1\n");
    cameraSelection.SetString(camera1.GetName());
    m_direction = -1;
  }
  else
  {
    printf("set camera 2\n");
    cameraSelection.SetString(camera2.GetName());
    m_direction = 1;
  }
}

void Robot::start_move(move_step_t *steps, int count)
{
    mv.steps = steps;
    mv.total_steps = count;

    m_step_start = m_timer.Get().value();
    step_n = 0;

    move_active = true;
    m_step = mv.steps[step_n];

    printf("%d: tv=%5.2f ts=%d\n", step_n+1, m_step_start, mv.total_steps);
    printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f f=%d d=%5.2f h=%5.2f\n", step_n+1,
        m_step.y, m_step.z,  m_step.t, m_step.intake, m_step.flywheel, m_step.distance, m_step.heading);
}

void Robot::evaluate_step(output_t &output)
{
  // TODO: this could be cleaner
  if (fabs(output.y) > .1 || fabs(output.z > .1))
  {
    move_active = false;

    // cancel drive/heading
    m_drive_active = false;
    m_heading_active = false;
  }

  if (!move_active) return;   // use output defaults
  
  double t = m_timer.Get().value();
  double elapsed = t - m_step_start;

  if (elapsed >= m_step.t)
  {
    // step timed out
    next_step(t);

    // done - TBD: short circuit, repeat last step, set step to 0s?
    if (!move_active) return;

    // cancel drive/heading
    // - TODO: start_step()?
    m_drive_active = false;
    m_heading_active = false;
  }

  // default outputs
  double y = m_step.y;
  double z = m_step.z;

  if (m_step.distance != 0 || m_step.heading != 0)
  {
    // there are one or more goals
    bool distance_complete = (m_step.distance == 0);
    if (!distance_complete)
    {
      y = drive_distance(m_step.distance, m_step.y > 0.0);
      z = drive_heading(0);
      distance_complete = (fabs(y) < 0.001 && fabs(z) < 0.001);
    }

    bool heading_complete = (m_step.heading == 0);
    if (!heading_complete)    // TODO: This doesn't work for absolute heading?!
    {
      z = drive_heading(m_step.heading, m_step.z > 0.0);
      heading_complete = (fabs(z) < 0.001 );
    }

    if (distance_complete && heading_complete)
    {
      // goals complete, advance regardless of time
      next_step(t);
    }
  }

  output.y = y;
  output.z = z;
  output.intake = m_step.intake;

  // flywheel select speeds from the dashboard
  switch (m_step.flywheel)
  {
    case 0: output.flywheel = 0.0; break;
    case 1: output.flywheel = m_fw_sp1; break;
    case 2: output.flywheel = m_fw_sp2; break;
    case 3: output.flywheel = m_fw_sp3; break;
    default: output.flywheel = 0.0; break;
  }

  if (m_step.pixy)
  {
    // get z from the pixy
    steer_pixy(output.z, m_alliance);
  }
}

void Robot::next_step(double t)
{
    if (move_active && step_n < mv.total_steps)
    {
      step_n += 1;
      if (step_n < mv.total_steps)
      {
          m_step_start = t;
          m_step = mv.steps[step_n];

          printf("%s: %d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f f=%d, d=%5.2f, h=%5.2f\n", __func__, step_n+1,
              m_step.y, m_step.z, m_step.t, m_step.intake, m_step.flywheel, m_step.distance, m_step.heading);
      }
      else
      {
        // sequence complete
        printf("%5.2f: move complete\n", t);

        move_active = false;
      }
    }
}

double Robot::drive_distance(double d, bool absolute)
{
    static double l_sp = 0.0;
    // static double r_sp = 0.0;

    double l_position = m_ll_encoder.GetPosition();
    // double r_position = m_rl_encoder.GetPosition();

    if (!m_drive_active)
    {
      l_sp = absolute ? d : d + l_position;
      printf("d=%5.2f lp=%5.2f sp=%5.2f\n", d, l_position, l_sp);
      // r_sp = absolute ? d : d + r_position;
      // printf("d=%5.2f rp=%5.2f sp=%5.2f\n", d, r_position, r_sp);
      m_drive_active = true;
    }

    double y = 0.0;
    double dd = l_sp - l_position;

    double sign = dd >= 0.0 ? 1.0 : -1.0;
    double magnitude = dd * sign;

    if (magnitude > 90)
      y = -0.4;
    else if (magnitude > 40)
      y = -0.3;
    else if (magnitude > 10)
      y = -0.2;
    else if (magnitude > 3)
      y = -0.15;
    else if (magnitude > .5)
      y = -0.1;
    else
    {
      // TODO: sync with heading termination
      y = 0.0;
      m_drive_active = false;
    }

    y *= sign;
    
    // printf("m=%5.2f y=%5.2f da=%d\n", magnitude, y, m_drive_active);

    return y;
}

double Robot::drive_heading(double h, bool absolute)
{
    static double m_heading_sp = 0;

    double currentAngle = m_gyro.GetAngle();

    if (!m_heading_active)
    {
      m_heading_sp = absolute ? h : h + currentAngle;
      m_heading_active = true;
      // printf("h=%5.2f a=%5.2f sp=%5.2f\n", h, currentAngle, m_heading_sp);
    }

    double z = 0;
    double da = m_heading_sp - currentAngle;

    double sign = da >= 0.0 ? 1.0 : -1.0;
    double da_magnitude = da * sign;

    if (da_magnitude > 30)
      z = -0.15;
    else if (da_magnitude > 10)
      z = -0.1;
    else if (da_magnitude > 1)
      z = -0.05;
    else
    {
      m_heading_active = false;
      z = 0;
    }

    z *= sign;

    // printf("a=%5.2f da=%5.2f z=%5.2f\n", currentAngle, da, z);

    return z;
}

bool Robot::steer_pixy(double &z, int color)
{
  bool result = false;

  int px, pa;
  get_Pixy_xy(px, pa, color);
  // printf("px=%d pa=%d color=%d\n", px, pa, color);

  if (color != 1 && color != 2) //blue or red not detected
  {
  } 
  else if(px > -160 && px < 160 && pa > 10)
  {
    // target found
    // printf("px=%d\n", px);
    z = ((double)px / 60) * 0.25;

    result = true;
  }

  return result;
}

void Robot::update_outputs(output_t output)
{
    // printf(">> y=%5.2f z=%5.2f\n", output.y, output.z);
    // printf("   in=%5.2f fw=%5.2f, l=%5.2f\n", output.intake, output.flywheel, output.lift);

    // update outputs regardless of state
    drive(output.y, output.z, true);

    m_intake_1.Set(output.intake);
    m_intake_2.Set(output.intake);
    m_lift.Set(output.lift);

#ifdef VELOCITY_CONTROL    
    m_pidController.SetReference(output.flywheel, rev::ControlType::kVelocity);
#else
    m_fw_motor.Set(-output.flywheel);
#endif
}

void Robot::drive(double y, double z, bool limit_accel)
{
    static double last_y = 0.0;
    static double last_z = 0.0;

    double tmp_y = last_y;
    double tmp_z = last_z;

    // limit acceleration
    double dy = y - last_y;
    double dz = z - last_z;

    if (limit_accel)
    {
      clamp(dy, accel_y_max, -accel_y_max);
      clamp(dz, accel_z_max, -accel_z_max);
    }

    last_y = last_y + dy;
    last_z = last_z + dz;

    if (tmp_y != last_y || tmp_z != last_z)
    {
      // printf("y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", y, last_y, z, last_z);
    }

    // positive z is counterclockwise
    m_robotDrive.ArcadeDrive(-last_y, last_z, false);
}
  
int Robot::get_Pixy_xy(int& x, int& a, int& colorSignature)
{
    //Pixy Camera:
    //sendPacket byte 0-1 16-bit sync, 2 type, 3 length of payload (len), 4-len payload
    //recvPacket byte 0-1 16-bit sync(174, 193), 2 type, 3 length of payload (len), 4-5 16-bit checksum, 6-len payload
    //byte 18 of the receiveBuffer is the tracking index. the nuber assigned to a newly found block.
    //byte 19 of the receiveBuffer is the age in frames. when it reaches 255 it stops tracking.

    //uint8_t setLamp[]={174,193,22,2,0,0};   //Turn off all LEDs including RGB
    //uint8_t setLamp[]={174,193,22,2,1,0};   //Turn on top LEDs
    //m_pixyPort.WriteBulk(setLamp, 6);

    uint8_t sig = colorSignature == 0 ? 1 : 2;
    uint8_t receiveBufffer[32];
    uint8_t getBlocks[]={174,193,32,2,sig,1};   //from the Pixy2 data sheet (sigmap 1,max blocks to return 1)
    m_pixyPort.WriteBulk(getBlocks, 6);       //from the frc::I2C class
    m_pixyPort.ReadOnly(32, receiveBufffer);  //from the frc::I2C class

    int xDirectionValue = receiveBufffer[8] | receiveBufffer[9]<<8;   //using two bytes. max is 315
    colorSignature = receiveBufffer[6];

    a = receiveBufffer[19];
    x = xDirectionValue - 160;

    // printf("pixy: index=%d x=%d color=%d\n", receiveBufffer[18], xDirectionValue, colorSignature);

    return true;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
