/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <cameraserver/CameraServer.h>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"


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
    int flywheel;
} move_step_t;

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

// shoot and back away
move_step_t mv_auto_1[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 5.0, 0.0, 1},
  // shoot
  {0.0, 0.0, 0.2, -0.3, 1},
  {0.0, 0.0, 1.5, 0.7, 1},
  // drive
  {-0.2, 0.0, 1.5, 0.0, 0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0},
};

// shoot, back up for cargo and return
move_step_t mv_auto_2[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0},
  // spin up
  {0.0, 0.0, 5.0, 0.0, 1},
  // shoot
  {0.0, 0.0, 0.2, -0.3, 1},
  {0.0, 0.0, 1.5, 0.7, 1},
  // drive
  // {0.4, 0.0, 2.5, 0.0, 1},
  {-0.2, 0.0, 2.2, 0.7, 0},
  {0.0, 0.0, 0.2, -0.3, 1},
  // reverse
  {0.2, 0.0, 2.2, 0.0, 1},
  {0, 0.0, 1.0, 0.7, 1},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0},
};

// shoot
move_step_t mv_shoot[] =
{
  // all cargo to the top
  {0.0, 0.0, 0.7, 0.5, 0},
  // back cargo away from flywheel
  {0.0, 0.0, 0.5, -0.3, 0},
  // spin up
  // {0.0, 0.0, 5.0, 0.0, 1},
  // // shoot 1
  // {0.0, 0.0, 0.3, 0.7, 1},
  // // recover
  // {0.0, 0.0, 1.0, 0.0, 1},
  // // shoot 2
  // {0.0, 0.0, 1.0, 0.7, 1},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0},
};

// spin up, back off
move_step_t mv_backoff[] =
{
  {-0.2, 0.0, 0.25, 0.0, 1},
  // {0.0, 0.0, 0.2, -0.3, 1},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0},
};

move_step_t none[] =
{
	{0.0, 0.0, 0.0, 0.0, 0},
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

	WPI_TalonSRX m_intake {3};

  WPI_VictorSPX m_lift {1};
  WPI_VictorSPX m_flywheel {2};

  // frc::PWMVictorSPX m_flywheel{3};
  // frc::PWMVictorSPX m_intake{1};
  // frc::PWMVictorSPX m_lift{6};

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

  frc::AnalogInput m_lift_position {0};
  frc::Timer m_timer;

  cs::UsbCamera camera1;
  cs::UsbCamera camera2;
  // cs::VideoSink server;

  nt::NetworkTableEntry cameraSelection;
  
  frc2::PIDController m_lift_pid {0.1, 0, 0};

  double m_fw_sp {0};

  double last_z {0};
  double last_y {0};

  double m_direction {-1};
  double accel_y_max {0.03};
  double accel_z_max {0.05};

  int m_lift_sp {0};
  
  double m_timestamp {0};
  
  size_t total_steps;
  size_t step;
  double t_step_end;
  
  // bool move_complete;
  bool step_complete;

  bool move_active {0};

  int m_camera {1};

 public:
  void RobotInit() {
    printf("Rapid React v3.4-np %s %s\n", __DATE__, __TIME__);

    frc::SmartDashboard::PutNumber("fw_sp",0);
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

    camera1 = frc::CameraServer::StartAutomaticCapture();
    // camera2 = frc::CameraServer::StartAutomaticCapture();

    camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    camera1.SetResolution(320, 240);
    camera1.SetFPS(15);

    // camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    // camera2.SetResolution(320, 240);
    // camera2.SetFPS(15);

    cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");

    // server = frc::CameraServer::AddServer("USB Camera 0");
    // server.SetSource(camera2);

    frc::SmartDashboard::PutNumber("delay", 2);
    frc::SmartDashboard::PutNumber("auto", 2);
    frc::SmartDashboard::PutNumber("fw_sp", 0.51);

    frc::SmartDashboard::PutNumber("P", 0.001);
  }

  void AutonomousInit() {
    m_timer.Start();
    
    double delay = frc::SmartDashboard::GetNumber("delay", 0);
    int auto_selection = frc::SmartDashboard::GetNumber("auto", 1);

    m_fw_sp = frc::SmartDashboard::GetNumber("fw_sp", 5);
    while (m_fw_sp < -1.0 || m_fw_sp > 1.0) {
      m_fw_sp /= 10;
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

    // move_complete = false;

    printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f fw=%d\n", step+1,
        mv.steps[0].y, mv.steps[0].z, mv.steps[0].t, mv.steps[0].intake, mv.steps[0].flywheel);
  }

  void AutonomousPeriodic() {
    update_move();

    double tmp_y = last_y;
    double tmp_z = last_z;

    double y = 0.0;
    double z = 0.0;

    // use move_active?

    if (step < mv.total_steps)
    {
        y = mv.steps[step].y;
        z = mv.steps[step].z;
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
        // printf("%5.2f: y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", t, y, last_y, z, last_z);
    }

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);

    m_intake.Set(mv.steps[step].intake);

    double fw_speed = 0;
    if (mv.steps[step].flywheel)
      fw_speed = m_fw_sp;

    m_flywheel.Set(-fw_speed);
  }

  void TeleopInit() {
    m_fw_sp = frc::SmartDashboard::GetNumber("fw_sp", 5);
    while (m_fw_sp < -1.0 || m_fw_sp > 1.0) {
      m_fw_sp /= 10;
    }

    printf("fw_speed=%5.2f\n,", m_fw_sp);
    // sync direction and camera selection
    set_camera_selection();

    double P = frc::SmartDashboard::GetNumber("P", 0);
    m_lift_pid.SetP(P);

    m_lift_sp = m_lift_position.GetValue();

    move_active = 0;
    m_timer.Start();
  }

  void TeleopPeriodic() {
    // select camera
    if (m_stick_d.GetRawButtonPressed(5))
    {
      m_camera = (m_camera == 1) ? 2 : 1;
      set_camera_selection();
    }

    // driver input
    double y = m_stick_d.GetRawAxis(1) * m_direction;
    double z = m_stick_d.GetRawAxis(4);

    scale(y, 0.15, 0.0, .4);
    scale(z, 0.15, 0.0, .2);    

    // printf("y=%5.2f z=%5.2f\n", y, z);
    
    // int lift_position = m_lift_position.GetValue();   
    // printf("pos=%d\n", lift_position);
    
    // double lift_speed = 0;
    double fw_speed = 0;
    double intake_speed = 0;

    if (m_stick_o.GetRawButtonPressed(7))
    {
      start_move(mv_shoot, SIZEOF_ARRAY(mv_shoot));
    }
    else if (m_stick_d.GetRawButtonPressed(1))
    {
      start_move(mv_backoff, SIZEOF_ARRAY(mv_backoff));
    }
    
    // auto movement
    if (move_active)
    {
      if (y >= -0.1 && y <= 0.1 && z >= -0.1 && z <= 0.1)
      {
        // no driver override
        update_move();

        y = mv.steps[step].y;
        z = mv.steps[step].z;

        if (mv.steps[step].flywheel == 1)
          fw_speed = m_fw_sp;
        else if (mv.steps[step].flywheel == 2)
          fw_speed = 0.4;
        
        intake_speed = mv.steps[step].intake;
      }
      else
      {
        // driver taking control, cancel automatic operation
        move_active = false;
      }
    }

    //flywheel motor control
    if (m_stick_d.GetRawButton(6))
    {
      fw_speed = m_fw_sp;
    }
    else if (m_stick_d.GetRawAxis(3) > 0.5)
    {
      fw_speed = 0.4;
    }

    if (m_stick_o.GetRawButton(2))
    {
      intake_speed = 0.5;
    }
    else if (m_stick_o.GetRawButton(3))
    {
      intake_speed = -0.3;
    }

    double lift_speed = 0.0;

    if (m_stick_o.GetRawButton(4))
    {
      // extend
      m_lift_sp = 1600;

      // if (lift_position < m_lift_sp) 
      lift_speed = 0.4; 
    }
    else if (m_stick_o.GetRawButton(1))
    {
      // retract
      m_lift_sp = 480;

      // if (lift_position > m_lift_sp) 
      lift_speed = -0.4;
    }

    // lift_speed = m_lift_pid.Calculate(lift_position, m_lift_sp);
    // printf("sp=%d pos=%d lift speed = %5.2f\n", m_lift_sp, lift_position, lift_speed2);
    clamp(lift_speed, 0.6, -0.6);

    // update the motors every pass
    drive(y, z);

    m_intake.Set(intake_speed);
    m_flywheel.Set(-fw_speed);
    m_lift.Set(lift_speed);
  }

  void DisabledInit() 
  {
    printf("disabled\n");

    m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }

  void DisabledPeriodic() {}

  void TestInit() {}
  void TestPeriodic() {}


  void start_move(move_step_t *move, int count)
  {
    mv.steps = move;
    mv.total_steps = count;

    auto tv = m_timer.Get();

    t_step_end = tv.value() + mv.steps[0].t;
    step = 0;

    move_active = true;

    // printf("%d: tv=%5.2f ts=%d y=%5.2f z=%5.2f t=%5.2f =%5.2f f=%d\n", step+1,
    //     tv.value(), mv.total_steps, mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].intake, mv.steps[step].flywheel);
  }

  void update_move()
  {
    if (!move_active) return;

    auto tv = m_timer.Get();

    // printf("%d: tv=%5.2f ts=%d y=%5.2f z=%5.2f t=%5.2f =%5.2f f=%d\n", step+1,
    //     tv.value(), mv.total_steps, mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].intake, mv.steps[step].flywheel);

    if (step < mv.total_steps)
    {
        step_complete = tv.value() > t_step_end;

        if (step_complete)
        {
          step += 1;
            if (step < mv.total_steps)
            {
                t_step_end += mv.steps[step].t;

                printf("%d: y=%5.2f z=%5.2f t=%5.2f =%5.2f f=%d\n", step+1,
                    mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].intake, mv.steps[step].flywheel);
            }
            else
            {
                // sequence complete
              printf("%5.2f: move complete\n", tv.value());

              step = mv.total_steps;
              move_active = false;
            }
        }
    }
  }

  void drive(double y, double z)
  {
    double tmp_y = last_y;
    double tmp_z = last_z;

    // limit acceleration
    double dy = y - last_y;
    double dz = z - last_z;

    clamp(dy, accel_y_max, -accel_y_max);
    clamp(dz, accel_z_max, -accel_z_max);

    last_y = last_y + dy;
    last_z = last_z + dz;

    if (tmp_y != last_y || tmp_z != last_z)
    {
      // printf("y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", y, last_y, z, last_z);
    }

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);
  }

  void set_camera_selection()
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
};


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
