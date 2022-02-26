/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>

#include <cameraserver/CameraServer.h>

#define SIZEOF_ARRAY(array_name) (sizeof(array_name) / sizeof(array_name[0]))

double rect_x;
double rect_y;
double rect_w;
double rect_h;

typedef struct
{
    double y;
    double z;
    double t;
    double intake;
    double discharge;
} move_step_t;

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

// back away
move_step_t mv_default[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // spin up
  {0.0, 0.0, 5.0, 0.0, 0.6},
  // shoot
  {0.0, 0.0, 1.5, 0.7, 0.6},
  // drive
  // {0.4, 0.0, 2.5, 0.0, 0.0},
  // {0.2, 0.0, 1.3, 0.0, 0.0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0},
};

move_step_t none[] =
{
	{0.0, 0.0, 0.0, 0.0, 0.0},
};

move_seq_t mv = {none, 0};


// static 
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

// static 
void scale(double &value, const double deadband, const double ll, const double ul)
{
  bool positive = (value >= 0.0);

  if (!positive) value *= -1;

//printf("1: value=%5.2f\n", value);

  // adjust for deadband
  if (value > deadband) value -= deadband;
  // else if (value < -deadband) value += deadband;
  else value = 0;

//printf("2: value=%5.2f\n", value);

  // scale based output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

//printf("3: value=%5.2f\n", value);

  // offset to minimum
  value += ll;
  // if (value > 0) value += ll;
  // else value -= ll;

  if (!positive) value *= -1;

//printf("4: value=%5.2f\n", value);
}

class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  frc::PWMVictorSPX m_right{0};
  frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_discharge{3};
  frc::PWMVictorSPX m_intake{4};
  frc::PWMVictorSPX m_lift{6};

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick_d {0};
  frc::Joystick m_stick_o {1};

  frc::AnalogInput m_pos_l1 {0};

  frc::Timer m_timer;

  double m_shooter {0};

  double last_z {0};
  double last_y {0};

  double m_direction {-1};
  double accel_max {0.05};

  int m_lift_sp {0};
  
  double m_timestamp {0};
  
  size_t total_steps;
  size_t step;
  double t_step_end;
  
  bool move_complete;
  bool step_complete;

 public:
  void RobotInit() {
    printf("Rapid React v2.0 %s %s\n", __DATE__, __TIME__);

    frc::SmartDashboard::PutNumber("Shooter",0);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();
    
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);

    m_rightLeadMotor.SetInverted(true);

    frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

    frc::SmartDashboard::PutNumber("delay", 3);
  }

  void AutonomousInit() {
    m_timer.Start();
    
    double delay = frc::SmartDashboard::GetNumber("delay", 5);

    mv.steps = mv_default;
    mv.total_steps = SIZEOF_ARRAY(mv_default);

    t_step_end = m_timer.Get().value() + delay;
    // t_step_end = m_timer.Get() + mv.steps[0].t;
    step = 0;

    move_complete = false;

    printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f d=%5.2f\n", step+1,
        mv.steps[0].y, mv.steps[0].z, mv.steps[0].t, mv.steps[0].intake, mv.steps[0].discharge);
  }

  void AutonomousPeriodic() {
    double tmp_y = last_y;
    double tmp_z = last_z;

    double t = (double)m_timer.Get().value();

    // printf("t=%5.2f\n", t);

    if (step < mv.total_steps)
    {
        step_complete = t > t_step_end;

        if (step_complete)
        {
          step += 1;
            if (step < mv.total_steps)
            {
                t_step_end += mv.steps[step].t;

                printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f d=%5.2f\n", step+1,
                    mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].intake, mv.steps[step].discharge);
            }
            else
            {
                // sequence complete
              printf("%5.2f: move complete\n", t);

              step = mv.total_steps;
              move_complete = true;
            }
        }
    }

    double y = 0.0;
    double z = 0.0;

    if (step < mv.total_steps)
    {
        y = mv.steps[step].y;
        z = mv.steps[step].z;
    }

    if (y == 0.0)
    {
        // set sign for pure rotations
        // z *= rotation;
    }

    // limit acceleration
    double dy = y - last_y;
    double dz = z - last_z;

    // clamp(dy, accel_max, -accel_max);
    // clamp(dz, accel_max, -accel_max);

    last_y = last_y + dy;
    last_z = last_z + dz;

    if (tmp_y != last_y || tmp_z != last_z)
    {
        printf("%5.2f: y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", t, y, last_y, z, last_z);
    }

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);

    m_intake.Set(mv.steps[step].intake);

    m_discharge.Set(mv.steps[step].discharge);
  }

  void TeleopInit() {
    m_shooter = frc::SmartDashboard::GetNumber("Shooter", 5);

    while (m_shooter < -1.0 || m_shooter > 1.0) {
      m_shooter /= 10;
    }
  }

  void TeleopPeriodic() {
    // driver input
    double y = m_stick_d.GetRawAxis(1) * m_direction;
    double z = m_stick_d.GetRawAxis(4);

    scale(y, 0.15, 0.0, .5);
    scale(z, 0.15, 0.0, .2);    

    int pos_l1 = m_pos_l1.GetValue();   

    printf("pos=%d\n", pos_l1);
    
    double lift_speed = 0;
    // motor control
    //discharge motor control
    if (m_stick_d.GetRawButton(1))
    {
      m_discharge.Set(-m_shooter);
    }
    else 
    {
      m_discharge.Set(0);
    }    

    if (m_stick_o.GetRawButton(2))
    {
        m_intake.Set(0.5);
    }
    else if (m_stick_o.GetRawButton(3))
    {
        m_intake.Set(-0.3);
    }
    else 
    {
      m_intake.Set(0);
    }

    if (m_stick_o.GetRawButton(1))
    {
      // extend
#if 0
        if (pos_l1 > 2400)
            lift_speed = -0.7;
        else if (pos_l1 > 2300)
            lift_speed = -0.3;
        else if (pos_l1 > 2200)
            lift_speed = -0.1;
        else
            lift_speed = 0;
#else
      lift_speed = -.6;
#endif
    }
    else if (m_stick_o.GetRawButton(4))
    {
      // retract
#if 0
        if (pos_l1 < 3500)
            lift_speed = 0.7;
        else if (pos_l1 < 3600 )
            lift_speed = 0.3;
        else if (pos_l1 < 3700)
            lift_speed = 0.1;
        else
            lift_speed = 0;
#else
      lift_speed = .6;
#endif
    }
    // printf("l1 speed=%5.2f\n", lift_speed);
    m_lift.Set(lift_speed);

    // update the motors every pass
    drive(y, z);
  }

  void DisabledInit() 
  {
    m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }

  void DisabledPeriodic() {}

  void TestInit() {}
  void TestPeriodic() {}

  void drive(double y, double z)
  {
    double tmp_y = last_y;
    double tmp_z = last_z;

    // limit acceleration
    double dy = y - last_y;
    double dz = z - last_z;

    clamp(dy, accel_max, -accel_max);
    clamp(dz, accel_max, -accel_max);

    last_y = last_y + dy;
    last_z = last_z + dz;

    if (tmp_y != last_y || tmp_z != last_z)
    {
      // printf("y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", y, last_y, z, last_z);
    }

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);
  }
};


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
