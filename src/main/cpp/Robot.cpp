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
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 4, rightLeadDeviceID = 3, rightFollowDeviceID = 2;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  frc::PWMVictorSPX m_right{0};
  frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_telescope{4};
  frc::PWMVictorSPX m_lift{6};
  frc::PWMVictorSPX m_winch{2};
  frc::PWMVictorSPX m_discharge{5};

  double m_shooter {0};

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick_d{0};
  frc::Joystick m_stick_o {1};

  frc::AnalogInput m_pos_l1 {0};
  
 public:
  void RobotInit() {
    printf("Rapid React v1.0 %s %s\n", __DATE__, __TIME__);

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
  }

 void TeleopInit() {
    m_shooter = frc::SmartDashboard::GetNumber("Shooter", 5);
    m_shooter = m_shooter / 10;
 }

  void TeleopPeriodic() {
    int pos_l1 = m_pos_l1.GetValue();   

    printf("pos=%d\n", pos_l1);

    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick_d.GetY(), m_stick_d.GetX());
     double lift_speed = 0;
    // motor control
    //discharge motor control
    if (m_stick_d.GetRawButton(1))
    {
      printf("pressed\n");
        m_discharge.Set(m_shooter);
    }
    else 
    {
//      printf("released\n");
      m_discharge.Set(0);
    }    

    if (m_stick_o.GetRawButton(2))
    {
//      printf("pressed\n");
        m_winch.Set(-.5);
    }
    else 
    {
//      printf("released\n");
      m_winch.Set(0);
    }    
    if (m_stick_o.GetRawButton(1))
    {
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
  }
};



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
