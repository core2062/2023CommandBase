// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeSpeedCommand.h"

IntakeSpeedCommand::IntakeSpeedCommand(IntakeSubsystem* intakeSubsystem, double intakeSpeed)
    : m_intakeSubsystem{intakeSubsystem},
      m_intakeSpeed{intakeSpeed},
      m_timeout{-1} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeSubsystem);
}
IntakeSpeedCommand::IntakeSpeedCommand(IntakeSubsystem* intakeSubsystem, double intakeSpeed, double timeout)
    : m_intakeSubsystem{intakeSubsystem},
      m_intakeSpeed{intakeSpeed},
      m_timeout{timeout} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeSubsystem);
}

void IntakeSpeedCommand::Initialize() {
  m_timer.Restart();
}

void IntakeSpeedCommand::Execute() {
  m_intakeSubsystem->SetIntakeMotors(m_intakeSpeed);
}

void IntakeSpeedCommand::End(bool interrupted) {
  m_intakeSubsystem->SetIntakeMotors(0);
}

bool IntakeSpeedCommand::IsFinished() {
  if (m_timeout != -1) {
    return m_timer.HasElapsed(units::time::second_t(m_timeout));
  } else {
    return false;
  }
}