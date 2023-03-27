// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"

DriveCommand::DriveCommand(DriveSubsystem* driveSubsystem, double distance)
    : m_driveSubsystem{driveSubsystem},
      m_distance{distance} {
  // Register that this command requires the subsystem.
  AddRequirements(m_driveSubsystem);
}

void DriveCommand::Initialize() {

}

void DriveCommand::Execute() {

}

