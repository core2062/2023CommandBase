// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeOffCommand.h"

IntakeOffCommand::IntakeOffCommand(IntakeSubsystem* subsystem)
    : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void IntakeOffCommand::Execute() {
  m_subsystem->SetIntakeMotor(0);
}