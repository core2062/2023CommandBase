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
  // m_chooser.SetDefaultOption("Mobility Routine", GetMobilityRoutine());
  // m_chooser.AddOption("Auto Balance Routine", GetAutoBalanceRoutine());
  // m_chooser.SetDefaultOption("Score Balance Routine", GetScoreBalanceRoutine());
  // m_chooser.AddOption("Score Mobility Routine", GetScoreMobilityRoutine());
  // frc::SmartDashboard::PutData("Autonomous", &m_chooser); 

  // m_chooser.SetDefaultOption("Set Comp Speed", SetCompDriveSpeed());
  // m_chooser.AddOption("Set Demo Speed", SetDemoDriveSpeed());
  // frc::SmartDashboard::PutData("Speed Selector", &m_chooser);

  m_chooser.AddOption("Set Comp Speed", SpeedOptions::COMP_SPEED);
  m_chooser.AddOption("Set Demo Speed", SpeedOptions::DEMO_SPEED);
  frc::SmartDashboard::PutData("Drive Speed Option", &m_chooser);

  m_chooser2.AddOption("Do Nothing", Autons::DO_NOTHING);
  m_chooser2.AddOption("Auto Balance", Autons::AUTOBALANCE);
  m_chooser2.AddOption("Mobility", Autons::MOBILITY);
  m_chooser2.AddOption("Score Balance", Autons::SCORE_AUTOBALANCE);
  m_chooser2.AddOption("Score Mobility", Autons::SCORE_MOBILITY);
  m_chooser2.SetDefaultOption("The Whole Shabang", Autons::THE_WHOLE_SHABANG);
  frc::SmartDashboard::PutData("Autonomous 2",&m_chooser2);

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
  frc2::JoystickButton{&m_driverController, 5}
      .OnTrue(&m_driveHalfSpeed)
      .OnFalse(&m_driveFullSpeed);

  frc2::JoystickButton{&m_driverController, 3}
    .OnTrue(new AutoBalanceCommand(&m_drive,1))
    .OnFalse(new AutoBalanceCommand(&m_drive,0));

  frc2::JoystickButton{&m_driverController, 4}
    .OnTrue(new AutoBalanceCommand(&m_drive,2))
    .OnFalse(new AutoBalanceCommand(&m_drive,0));

  // (frc2::JoystickButton{&m_driverController, 3} || frc2::JoystickButton{&m_driverController, 4})
  //   .OnFalse(new AutoBalanceCommand(&m_drive,0));

  frc2::JoystickButton{&m_operatorController,1} // A Button
    .OnTrue(&m_intakePositionToggle); // Toggles intake position

  (frc2::JoystickButton(&m_operatorController,2) || frc2::JoystickButton(&m_operatorController,3) || frc2::JoystickButton(&m_operatorController,4) || frc2::JoystickButton(&m_operatorController,6))
    .OnFalse(new IntakeSpeedCommand(&m_intake,0));

  frc2::JoystickButton(&m_operatorController,3) // Y button
    .OnTrue(new IntakeSpeedCommand(&m_intake,0.25)); // Score low

  frc2::JoystickButton(&m_operatorController,4) // X Button
    .OnTrue(new IntakeSpeedCommand(&m_intake,0.6)); // Score med

  frc2::JoystickButton(&m_operatorController,2) // B Button
    .OnTrue(new IntakeSpeedCommand(&m_intake,1)); // Score high

  frc2::JoystickButton(&m_operatorController,6) // RB
    .OnTrue(new IntakeSpeedCommand(&m_intake,-0.50)); // Intake in

  // frc2::JoystickButton{&m_driverController,1}
  // .OnTrue(new DriveCommand(&m_drive,-1));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  
  // // Create a voltage constraint to ensure we don't accelerate too fast
  // frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
  //     frc::SimpleMotorFeedforward<units::meters>{
  //         DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
  //     DriveConstants::kDriveKinematics, 10_V};

  // // Set up config for trajectory
  // frc::TrajectoryConfig config{AutoConstants::kMaxSpeed,
  //                              AutoConstants::kMaxAcceleration};
  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(DriveConstants::kDriveKinematics);
  // // Apply the voltage constraint
  // config.AddConstraint(autoVoltageConstraint);

  // // An example trajectory to follow.  All units in meters.
  // // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  // //     // Start at the origin facing the +X direction
  // //     frc::Pose2d{0_m, 0_m, 0_deg},
  // //     // Pass through these two interior waypoints, making an 's' curve path
  // //     {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
  // //     // End 0.6 meters straight ahead of where we started, facing forward
  // //     frc::Pose2d{3_m, 0_m, 0_deg},
  // //     // Pass the config
  // //     config);

  
  // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  // deployDirectory = deployDirectory / "paths" / "BlueMoveBack.wpilib.json";
  // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  // frc2::RamseteCommand ramseteCommand{
  //     trajectory,
  //     [this]() { return m_drive.GetPose(); },
  //     frc::RamseteController{AutoConstants::kRamseteB,
  //                            AutoConstants::kRamseteZeta},
  //     frc::SimpleMotorFeedforward<units::meters>{
  //         DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
  //     DriveConstants::kDriveKinematics,
  //     [this] { return m_drive.GetWheelSpeeds(); },
  //     frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
  //     frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
  //     [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
  //     {&m_drive}};

  // // Reset odometry to the starting pose of the trajectory.
  // m_drive.ResetOdometry(trajectory.InitialPose());
  
  // // no auto
  // return new frc2::SequentialCommandGroup(
  //     std::move(ramseteCommand),
  //     frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
  //     std::move(AutoBalanceCommand{&m_drive,1}),
  //     std::move(DelayCommand(&m_drive,1.0_s)),
  //     std::move(AutoBalanceCommand{&m_drive,2}),
  //     frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));

  // return new frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([this] { m_drive.SetNeutralMode(NeutralMode::Brake); }, {}),
  //   std::move(ramseteCommand),
  //   frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {})); 
  
  // return m_chooser.GetSelected();

  SpeedOptions speedChoice = m_chooser.GetSelected();
  switch(speedChoice)
  {
  case SpeedOptions::COMP_SPEED:
    return SetCompDriveSpeed();
    break;
  case SpeedOptions::DEMO_SPEED:
    return SetDemoDriveSpeed();
    break;
  }

  Autons choice = m_chooser2.GetSelected();
  switch (choice)
  {
  case Autons::AUTOBALANCE:
    return GetAutoBalanceRoutine();
    break;
  case Autons::SCORE_AUTOBALANCE:
    return GetScoreBalanceRoutine();
    break;
  case Autons::SCORE_MOBILITY:
    return GetScoreMobilityRoutine();
    break;
  case Autons::MOBILITY:
    return GetMobilityRoutine();
    break;
  case Autons::THE_WHOLE_SHABANG:
    return GetTheWholeShabang();
    break;
  default:
    return new frc2::SequentialCommandGroup(frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
    break;
  }

}

frc2::Command* RobotContainer::SetCompDriveSpeed() {
  std::cout << "Setting Comp Drive Speed" << endl;
  SetArcadeDriveSpeedModifier(0.8);
}

frc2::Command* RobotContainer::SetDemoDriveSpeed() {
  std::cout << "Setting Comp Drive Speed" << endl;
  SetArcadeDriveSpeedModifier(0.3);
}


frc2::Command* RobotContainer::GetAutoBalanceRoutine() {
  std::cout << "In GetAutoBalanceRoutine()" << endl;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "BlueMoveBack.wpilib.json";
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc2::RamseteCommand ramseteCommand{
      trajectory,
      [this]() { return m_drive.GetPose(); },
      frc::RamseteController{AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right);
                                      std::cout << "Driving in autobalance" << std::endl; },
      {&m_drive}};

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(trajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(

  std::move(ramseteCommand),
  frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
  std::move(AutoBalanceCommand{&m_drive,1}),
  std::move(DelayCommand(&m_drive,1.0_s)),
  std::move(AutoBalanceCommand{&m_drive,2}),
  frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));


  // return &ramseteCommand;
}

frc2::Command* RobotContainer::GetAutoBalanceRoutine2() {
  // // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
      DriveConstants::kDriveKinematics, 10_V};

  // // Set up config for trajectory
  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration};
  // // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // // An example trajectory to follow.  All units in meters.
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 0.6 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  
  // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  // deployDirectory = deployDirectory / "paths" / "BlueMoveBack.wpilib.json";
  // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc2::RamseteCommand ramseteCommand{
      trajectory,
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

  // // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(trajectory.InitialPose());
  
  // // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
      std::move(AutoBalanceCommand{&m_drive,1}),
      std::move(DelayCommand(&m_drive,1.0_s)),
      std::move(AutoBalanceCommand{&m_drive,2}),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));

  // return new frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([this] { m_drive.SetNeutralMode(NeutralMode::Brake); }, {}),
  //   std::move(ramseteCommand),
  //   frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {})); 
}

frc2::Command* RobotContainer::GetScoreBalanceRoutine() {
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "BlueMoveBack.wpilib.json";
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc2::RamseteCommand ramseteCommand{
      trajectory,
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
  m_drive.ResetOdometry(trajectory.InitialPose());

  return new frc2::SequentialCommandGroup(
    std::move(IntakeSpeedCommand(&m_intake, 1, 1)),
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
    std::move(AutoBalanceCommand{&m_drive,1}),
    std::move(DelayCommand(&m_drive,1.0_s)),
    std::move(AutoBalanceCommand{&m_drive,2}),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));

    // return &ramseteCommand;
}

frc2::Command* RobotContainer::GetScoreMobilityRoutine() {
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "MoveMobility.wpilib.json";
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());


// frc2::RamseteCommand ram{}
  frc2::RamseteCommand ramseteCommand{
      trajectory,
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

  m_drive.ResetOdometry(trajectory.InitialPose());
      
      return new frc2::SequentialCommandGroup(
        std::move(IntakeSpeedCommand(&m_intake, 1, 1)),
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));

    // return &ramseteCommand;
}

frc2::Command* RobotContainer::GetMobilityRoutine() {
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "MoveMobility.wpilib.json";
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());


// frc2::RamseteCommand ram{}
  frc2::RamseteCommand ramseteCommand{
      trajectory,
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
      
  m_drive.ResetOdometry(trajectory.InitialPose());

  return new frc2::SequentialCommandGroup(
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));

    // return &ramseteCommand;
}

frc2::Command* RobotContainer::GetTheWholeShabang() {
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "MoveMobilityToBalance.wpilib.json";
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());


// frc2::RamseteCommand ram{}
  frc2::RamseteCommand ramseteCommand{
      trajectory,
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
      
  m_drive.ResetOdometry(trajectory.InitialPose());

  deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "MobilityToBalance.wpilib.json";
  frc::Trajectory trajectory2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc2::RamseteCommand ramseteCommand2{
      trajectory2,
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

  return new frc2::SequentialCommandGroup(
    std::move(IntakeSpeedCommand(&m_intake, 1, 1)),
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
    std::move(ramseteCommand2),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
    std::move(AutoBalanceCommand{&m_drive,1}),
    std::move(DelayCommand(&m_drive,1.0_s)),
    std::move(AutoBalanceCommand{&m_drive,2}),
    frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}

void RobotContainer::Feed() {
  m_drive.Feed();
}

void RobotContainer::SetArcadeDriveSpeedModifier(double speed) {
  m_drive.SetDriveSpeedModifier(speed);
}