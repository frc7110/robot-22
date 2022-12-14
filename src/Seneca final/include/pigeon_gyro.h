// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <hal/SimDevice.h>

// #include "frc/GyroBase.h"
#include "frc/interfaces/Gyro.h"
//#include "frc/SPI.h"

#include "ctre/Phoenix.h"


namespace frc {

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 *
 * The Gyro class tracks the robots heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a
 * short calibration routine where it samples the gyro while at rest to
 * determine the default offset. This is subtracted from each sample to
 * determine the heading.
 *
 * This class is for the digital ADXRS450 gyro sensor that connects via SPI.
 * Only one instance of an ADXRS Gyro is supported.
 */
class pigeon_gyro : public Gyro {
 public:
  /**
   * Gyro constructor on onboard CS0.
   */
  pigeon_gyro(int m_can_id);
  pigeon_gyro(ctre::phoenix::motorcontrol::can::TalonSRX& talon);

  /**
   * Gyro constructor on the specified SPI port.
   *
   * @param port The SPI port the gyro is attached to.
   */
//explicit pigeon_gyro(int m_can_id);

  ~pigeon_gyro() override = default;

  pigeon_gyro(pigeon_gyro&&) = default;
  pigeon_gyro& operator=(pigeon_gyro&&) = default;

  /**
   * Return the actual angle in degrees that the robot is currently facing.
   *
   * The angle is based on integration of the returned rate from the gyro.
   * The angle is continuous, that is it will continue from 360->361 degrees.
   * This allows algorithms that wouldn't want to see a discontinuity in the
   * gyro output as it sweeps from 360 to 0 on the second time around.
   *
   * @return the current heading of the robot in degrees.
   */
  double GetAngle() const override;

  /**
   * Return the rate of rotation of the gyro
   *
   * The rate is based on the most recent reading of the gyro.
   *
   * @return the current rate in degrees per second
   */
  double GetRate() const override;

  /**
   * Reset the gyro.
   *
   * Resets the gyro to a heading of zero. This can be used if there is
   * significant drift in the gyro and it needs to be recalibrated after it has
   * been running.
   */
  void Reset() override;

  /**
   * Initialize the gyro.
   *
   * Calibrate the gyro by running for a number of samples and computing the
   * center value. Then use the center value as the Accumulator center value for
   * subsequent measurements.
   *
   * It's important to make sure that the robot is not moving while the
   * centering calculations are in progress, this is typically done when the
   * robot is first turned on while it's sitting at rest before the competition
   * starts.
   */
  void Calibrate() final;

  /**
   * Get the SPI port number.
   *
   * @return The SPI port number.
   */
  int GetPort() const;

 private:
  int m_can_id;

  PigeonIMU * _pidgey;

  mutable double m_angle;
  mutable double m_rate;

  hal::SimDevice m_simDevice;
  hal::SimDouble m_simAngle;
  hal::SimDouble m_simRate;
};

}  // namespace frc
