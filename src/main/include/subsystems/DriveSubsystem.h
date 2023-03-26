// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>
#include <ctre/phoenix/motorcontrol/GroupMotorControllers.h>

#include "Constants.h"

enum class DriveSide{LEFT = 1, RIGHT = 2, BOTH = 3};

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double GetAverageEncoderDistance();

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // frc::Encoder& GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // frc::Encoder& GetRightEncoder();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Gets the distance driven in meters
   * 
   * @param sensorCounts Then encoder value
  */
  units::meter_t NativeUnitsToDistanceMeters(double sensorCounts);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // frc::PWMSparkMax m_leftPrimary;
  // frc::PWMSparkMax m_leftSecondary;
  // frc::PWMSparkMax m_rightPrimary;
  // frc::PWMSparkMax m_rightSecondary;
  WPI_TalonSRX m_leftPrimary{3};
  WPI_TalonSRX m_leftSecondary{4};
  WPI_TalonSRX m_rightPrimary{1};
  WPI_TalonSRX m_rightSecondary{2};

  // The motors on the left side of the drive
  frc::MotorControllerGroup m_leftMotors{m_leftPrimary, m_leftSecondary};

  // The motors on the right side of the drive
  frc::MotorControllerGroup m_rightMotors{m_rightPrimary, m_rightSecondary};

  // The robot's drive
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  // The left-side drive encoder
  // frc::Encoder m_leftEncoder;

  // The right-side drive encoder
  // frc::Encoder m_rightEncoder;

  // The gyro sensor
  AHRS m_gyro{frc::SerialPort::kUSB};


  // Odometry class for tracking robot pose
  frc::DifferentialDriveOdometry m_odometry;
};
