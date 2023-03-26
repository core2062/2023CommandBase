// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeInCommand.h"

IntakeInCommand::IntakeInCommand(IntakeSubsystem* subsystem)
    : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void IntakeInCommand::Execute() {
  m_subsystem->SetIntakeMotor(1);
}