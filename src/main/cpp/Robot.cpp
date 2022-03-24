// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#define SIZEOF_ARRAY(array_name) (sizeof(array_name) / sizeof(array_name[0]))

static const char *version = "Rapid React v5.2x";

static const int key_alt = 6;     // RB
static const int key_retract = 1; // A
static const int key_extend = 4;  // Y

// y, z, t, intake, fw, d, h, pixy

// shoot and back away
move_step_t mv_auto_1[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // shoot
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // drive
  {-0.2, 0.0, 1.5, 0.0, 0, 0.0, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// shoot, back up for cargo and return
move_step_t mv_auto_2[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // back up with intake running
  // {0.4, 0.0, 2.5, 0.0, 1, 0.0, 0.0, 0},
  {-0.22, 0.0, 2.5, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // reverse
  {0.22, 0.0, 2.4, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// shoot, back up for cargo and return
move_step_t mv_auto_3[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 1.5, 0.7, 1, 0.0, 0.0, 0},
  // back up with intake running
  // {0.4, 0.0, 2.5, 0.0, 1, 0.0, 0.0, 0},
  {-0.22, 0.0, 2.5, 0.7, 0, 0.0, 0.0, 0},
  // "chamber" the cargo
  {0.0, 0.0, 0.2, -0.3, 1, 0.0, 0.0, 0},
  // reverse
  {0.22, 0.0, 2.4, 0.0, 1, 0.0, 0.0, 0},
  // fire
  {0, 0.0, 1.0, 0.7, 1, 0.0, 0.0, 0},
  // back up with intake running
  // {0.4, 0.0, 2.5, 0.0, 1, 0.0, 0.0, 0},
  {0.0, 0.0, 5.5, 0.7, 0, 0.0, -45.0, 0},
  {-0.22, 0.0, 5.5, 0.7, 0, 0.0, 0.0, 1},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// shoot
move_step_t mv_shoot_low[] =
{
  // all cargo to the top
  {0.0, 0.0, 0.7, 0.5, 0, 0.0, 0.0, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.3, -0.3, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.0, 0.0, 2, 0.0, 0.0, 0},
  // shoot 1
  {0.0, 0.0, 0.3, 0.7, 2, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 0.5, 0.0, 2, 0.0, 0.0, 0},
  // shoot 2
  {0.0, 0.0, 0.3, 0.7, 2, 0.0, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// back up
move_step_t mv_back[] =
{
  // back up
  {0.0, 0.0, 0.7, 0.0, 0, 12, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// back up
move_step_t mv_back_shot[] =
{
  // back up
  {0.0, 0.0, 0.7, 0.0, 0, 12, 0.0, 0},
  // all cargo to the top
  {0.0, 0.0, 0.5, 0.5, 0, 0.0, 0.0, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.6, -0.3, 0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 1.2, 0.0, 1, 0.0, 0.0, 0},
  // shoot 1
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
  // recover
  {0.0, 0.0, 0.7, 0.0, 1, 0.0, 0.0, 0},
  // shoot 2
  {0.0, 0.0, 0.3, 0.7, 1, 0.0, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// drive
move_step_t mv_distance[] =
{
  // drive distance
  {-0.2, 0.0, 5.0, 0.0, 0, 60.0, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
};

// turn
move_step_t mv_heading[] =
{
  // drive distance
  {0.0, 0.0, 5.0, 0.0, 0, 0.0, -45.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0, 0.0, 0.0, 0},
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

  // scale based output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

  //printf("3: value=%5.2f\n", value);

  // offset to minimum
  value += ll;

  value *= sign;  // restore sign

  //printf("4: value=%5.2f\n", value);
}


void Robot::RobotInit() 
{
    printf("%s %s %s\n", version, __DATE__, __TIME__);

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

    m_gyro.Reset();

    camera1 = frc::CameraServer::StartAutomaticCapture();
    camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera1.SetResolution(320, 240);
    // camera1.SetResolution(320, 180);  // Lifecam supports 16:9?
    camera1.SetFPS(15);

#if 0
    camera2 = frc::CameraServer::StartAutomaticCapture();
    camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera2.SetResolution(320, 240);
    // camera2.SetResolution(320, 180);  // Lifecam supports 16:9?
    camera2.SetFPS(15);
#endif

    cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");

    // sync direction and camera selection
    set_camera_selection();

    frc::SmartDashboard::PutNumber("delay", 0);
    frc::SmartDashboard::PutNumber("auto", 2);
    frc::SmartDashboard::PutNumber("fw_sp1", 0.55);
    frc::SmartDashboard::PutNumber("fw_sp2", 0.35);

    frc::SmartDashboard::PutNumber("P", 0.002);

    m_alliance = frc::DriverStation::GetAlliance();

    m_timer.Start();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
    m_timer.Start();

    double delay = frc::SmartDashboard::GetNumber("delay", 0);
    int auto_selection = frc::SmartDashboard::GetNumber("auto", 1);

    m_fw_sp1 = frc::SmartDashboard::GetNumber("fw_sp1", 0);
    while (m_fw_sp1 < -1.0 || m_fw_sp1 > 1.0) {
      m_fw_sp1 /= 10;
    }

    m_fw_sp2 = frc::SmartDashboard::GetNumber("fw_sp2", 0);
    while (m_fw_sp2 < -1.0 || m_fw_sp2 > 1.0) {
      m_fw_sp2 /= 10;
    }

    printf("auto=%d\n", auto_selection);
    if (auto_selection == 1)
    {
      mv_auto_1[0].t = delay;
      start_move(mv_auto_1, SIZEOF_ARRAY(mv_auto_1));
    }
    else if (auto_selection == 2)
    {
      mv_auto_2[0].t = delay;
      start_move(mv_auto_2, SIZEOF_ARRAY(mv_auto_2));
    }
    else if (auto_selection == 3)
    {
      mv_auto_2[0].t = delay;
      start_move(mv_auto_3, SIZEOF_ARRAY(mv_auto_3));
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
    m_fw_sp1 = frc::SmartDashboard::GetNumber("fw_sp1", 5);
    while (m_fw_sp1 < -1.0 || m_fw_sp1 > 1.0) {
      m_fw_sp1 /= 10;
    }

    m_fw_sp2 = frc::SmartDashboard::GetNumber("fw_sp2", 0);
    while (m_fw_sp2 < -1.0 || m_fw_sp2 > 1.0) {
      m_fw_sp2 /= 10;
    }

    m_alliance = frc::DriverStation::GetAlliance();

    printf("fw_speed=%5.2f/%5.2f\n", m_fw_sp1, m_fw_sp2);

    double P = frc::SmartDashboard::GetNumber("P", 0);
    m_lift_pid.SetP(P);

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

    // select camera
    if (m_stick_d.GetRawButtonPressed(5))
    {
      m_camera = (m_camera == 1) ? 2 : 1;
      set_camera_selection();
    }

    // driver input
    double y = m_stick_d.GetRawAxis(1) * m_direction;
    double z = m_stick_d.GetRawAxis(4);

    if (m_lift_sp < 1000)
    {
      scale(y, 0.05, 0.0, .6);
      scale(z, 0.05, 0.0, .3);
    }
    else
    {
      scale(y, 0.05, 0.0, .325);
      scale(z, 0.05, 0.0, .25);
    }
  
    output.y = y;
    output.z = z;

#if 0
    double ll_pos = m_ll_encoder.GetPosition();
    double rl_pos = m_rl_encoder.GetPosition();
    printf("p=%5.2f/%5.2f\n", ll_pos, rl_pos);
#endif
#if 0
    double ll_velocity = m_ll_encoder.GetVelocity();
    double rl_velocity = m_rl_encoder.GetVelocity();
    printf("v=%5.2f/%5.2f\n", ll_velocity, rl_velocity);
#endif

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
      intake_speed = 0.5;
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
        m_lift_sp = 1340;
      }
      else if (m_stick_o.GetRawButtonPressed(key_retract))
      {
        m_lift_sp = 400;
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
      start_move(mv_shoot_low, SIZEOF_ARRAY(mv_shoot_low));
    }
    else if (m_stick_d.GetRawButtonPressed(3))
    {
      start_move(mv_back_shot, SIZEOF_ARRAY(mv_back_shot));
    }
#if 0
    if (m_stick_o.GetRawButtonPressed(4))
    {
      start_move(mv_distance, SIZEOF_ARRAY(mv_distance));
    }
    else if (m_stick_o.GetRawButtonPressed(2))
    {
      start_move(mv_heading, SIZEOF_ARRAY(mv_heading));
    }

    // abort move?
    if (m_stick_o.GetRawButtonPressed(1))
    {
      move_active = false;
    }
#endif

    evaluate_step(output);

    if (m_stick_d.GetRawAxis(2) > 0.5)
    {
        // get z from the pixy
        steer_pixy(output.z, m_alliance);
    }

    // all motors should be updated every pass
    update_outputs(output);
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
  if (!move_active) return;   // use output defaults

  double t = m_timer.Get().value();
  double elapsed = t - m_step_start;

  // printf("elapsed=%5.2f\n", elapsed);
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

#if 0
  // TBD else if?
  if (m_step.distance == 0 && m_step.heading == 0)
  {
    // no goals, time based only
    output.y = y;
    output.z = z;
    output.intake = m_step.intake;
    if (m_step.flywheel) output.flywheel = (m_step.flywheel) ? m_fw_sp1 : m_fw_sp2;
  }
  else
  {
    // may have one or more goals
    bool distance_complete = (m_step.distance == 0);
    if (!distance_complete)
    {
      y = drive_distance(m_step.distance);
      z = drive_heading(0);
      distance_complete = (fabs(y) < 0.001 && fabs(z) < 0.001);
    }

    bool heading_complete = (m_step.heading != 0);
    if (!heading_complete)    // TODO: This doesn't work for absolute heading?!
    {
      z = drive_heading(m_step.heading);
      heading_complete = (fabs(z) < 0.001 );
    }

    if (distance_complete && heading_complete)
    {
      // goal complete, advance regardless of time
      next_step(t);
    }
    else
    {
      output.y = y;
      output.z = z;
      output.intake = m_step.intake;
      if (m_step.flywheel) output.flywheel = (m_step.flywheel == 1) ? m_fw_sp1 : m_fw_sp2;

      if (m_step.pixy)
      {
        // get z from the pixy
        steer_pixy(output.z, m_alliance);
      }
    }
  }
#else
  if (m_step.distance != 0 || m_step.heading != 0)
  {
    // there are one or more goals
    bool distance_complete = (m_step.distance == 0);
    if (!distance_complete)
    {
      y = drive_distance(m_step.distance);
      z = drive_heading(0);
      distance_complete = (fabs(y) < 0.001 && fabs(z) < 0.001);
    }

    bool heading_complete = (m_step.heading == 0);
    if (!heading_complete)    // TODO: This doesn't work for absolute heading?!
    {
      z = drive_heading(m_step.heading);
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
  if (m_step.flywheel) output.flywheel = (m_step.flywheel == 1) ? m_fw_sp1 : m_fw_sp2;

  if (m_step.pixy)
  {
    // get z from the pixy
    steer_pixy(output.z, m_alliance);
  }
#endif
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

    if (magnitude > 30)
      y = -0.4;
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
      printf("h=%5.2f a=%5.2f sp=%5.2f\n", h, currentAngle, m_heading_sp);
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
  printf("px=%d pa=%d color=%d\n", px, pa, color);

  if (color != 1 && color != 2) //blue or red not detected
  {
  } 
  else if(px > -160 && px < 160 && pa > 10)
  {
    // target found
    printf("px=%d\n", px);
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
    drive(output.y, output.z, false);

    m_intake_1.Set(output.intake);
    m_intake_2.Set(output.intake);
    m_flywheel.Set(-output.flywheel);
    m_lift.Set(output.lift);
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

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);
}
  
int Robot::get_Pixy_xy(int& x, int& a, int& colorSignature)
{
#if 0
  struct header_s
  {
    uint16_t sync;
    uint8_t type;
    uint8_t length;
    uint16_t checksum;
  };

  struct block_s
  {
    uint16_t m_signature;
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_width;
    uint16_t m_height;
    int16_t m_angle;
    uint8_t m_index;
    uint8_t m_age;
  };

  struct buffer_s
  {
    struct header_s header;
    struct block_s payload;
  } buffer;
#endif

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

    printf("pixy: index=%d x=%d color=%d\n", receiveBufffer[18], xDirectionValue, colorSignature);

    // frc::SmartDashboard::PutString("DB/String 5", "Index: "+ std::to_string(receiveBufffer[18]));
    // frc::SmartDashboard::PutString("DB/String 6", "Age: "+ std::to_string(a));

    return true;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
