// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"

DriveCommand::DriveCommand(DriveSubsystem* driveSubsystem, string pathName)
    : m_driveSubsystem{driveSubsystem},
      m_pathName{pathName} {
  // Register that this command requires the subsystem.
  AddRequirements(m_driveSubsystem);
  
  // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  // deployDirectory = deployDirectory / "paths" / "YourPath.wpilib.json";
  // trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
}

void DriveCommand::Initialize() {
}

void DriveCommand::Execute() {
}

void DriveCommand::End(bool interrupted) {
}

bool DriveCommand::IsFinished() {
  return true;
}