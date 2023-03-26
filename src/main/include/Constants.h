// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
constexpr int kLeftMotor1Port = 3;
constexpr int kLeftMotor2Port = 4;
constexpr int kRightMotor1Port = 1;
constexpr int kRightMotor2Port = 2;
constexpr int kIntakeMotorPort = 5;

// constexpr int kLeftEncoderPorts[]{0, 1};
// constexpr int kRightEncoderPorts[]{2, 3};
// constexpr bool kLeftEncoderReversed = false;
// constexpr bool kRightEncoderReversed = true;

// TODO: determine track Width
constexpr auto kTrackwidth = 0.6096_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr int kEncoderCPR = 2049;
constexpr double kGearRatio = 12.86; // 12.86:1 ratio
constexpr double kWheelDiameter = static_cast<double>(0.1524_m);
// TODO: Calculate kEncoderDistancePerPulse
// constexpr double kEncoderDistancePerPulse =
//     // Assumes the encoders are directly mounted on the wheel shafts
//     (kWheelDiameterInches * std::numbers::pi) /
//     static_cast<double>(kEncoderCPR);
constexpr double kEncoderDistancePerPulse =
        (kWheelDiameter * std::numbers::pi) / (static_cast<double>(kEncoderCPR)/kGearRatio);

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 0.30893_V;
constexpr auto kv = 1.8004 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.29226 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPDriveVel = 0.13201;
}  // namespace DriveConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 1_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants