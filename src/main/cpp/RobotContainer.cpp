// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here


  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {// TODO: Figure out which axis on controller are actually being used (they should actually be correct)
        m_drive.ArcadeDrive(-m_driverController.GetRawAxis(1), // Left Y
                            -m_driverController.GetRawAxis(4)); // Right X
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton{&m_driverController, 6}
      .OnTrue(&m_driveHalfSpeed)
      .OnFalse(&m_driveFullSpeed);

  frc2::JoystickButton{&m_driverController, 4}
    .OnTrue(new AutoBalanceCommand(&m_drive,1));

  frc2::JoystickButton{&m_driverController, 3}
    .OnTrue(new AutoBalanceCommand(&m_drive,2));

  // frc2::JoystickButton{&m_operatorController,1} // A Button
  //     .ToggleOnTrue(&m_intakePositionToggle); // Toggles intake position

  (frc2::JoystickButton(&m_operatorController,2) || frc2::JoystickButton(&m_operatorController,3) || frc2::JoystickButton(&m_operatorController,4) || frc2::JoystickButton(&m_operatorController,6))
    .OnFalse(new IntakeSpeedCommand(&m_intake,0));

  frc2::JoystickButton(&m_operatorController,4) // Y button
    .OnTrue(new IntakeSpeedCommand(&m_intake,0.25)); // Score low

  frc2::JoystickButton(&m_operatorController,3) // X Button
    .OnTrue(new IntakeSpeedCommand(&m_intake,0.5)); // Score med

  frc2::JoystickButton(&m_operatorController,2) // B Button
    .OnTrue(new IntakeSpeedCommand(&m_intake,1)); // Score high

  frc2::JoystickButton(&m_operatorController,6) // RB
    .OnTrue(new IntakeSpeedCommand(&m_intake,-0.25)); // Intake in
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  /*
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
      DriveConstants::kDriveKinematics, 10_V};

  // Set up config for trajectory
  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{0.2_m, 0.2_m}, frc::Translation2d{0.4_m, -0.2_m}},
      // End 0.6 meters straight ahead of where we started, facing forward
      frc::Pose2d{0.6_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand{
      exampleTrajectory,
      [this]() { return m_drive.GetPose(); },
      frc::RamseteController{AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive}};

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {})); */
  
  return m_chooser.GetSelected();
}