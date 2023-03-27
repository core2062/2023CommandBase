// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeSpeedCommand.h"

IntakeSpeedCommand::IntakeSpeedCommand(IntakeSubsystem* intakeSubsystem, double intakeSpeed)
    : m_intakeSubsystem{intakeSubsystem},
      m_intakeSpeed{intakeSpeed} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeSubsystem);
}

void IntakeSpeedCommand::Execute() {
  m_intakeSubsystem->SetIntakeMotors(m_intakeSpeed);
}