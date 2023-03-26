// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

using namespace DriveConstants;

IntakeSubsystem::IntakeSubsystem() : m_intakeMotor{kIntakeMotorPort} {
  // Implementation of subsystem constructor goes here.
  m_intakeSpeed = 0.5;
}

frc2::CommandPtr IntakeSubsystem::IntakeMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  SmartDashboard::PutBoolean("Intake Up",m_intakeUp);
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void IntakeSubsystem::SetIntakeMotor(IntakeDirection direction){
  switch (direction)
  {
  case IntakeDirection::OUT:
    m_intakeMotor.Set(ControlMode::PercentOutput,m_intakeSpeed);
    break;
  case IntakeDirection::IN:
    m_intakeMotor.Set(ControlMode::PercentOutput,m_intakeSpeed*-1);
  default:
    m_intakeMotor.Set(ControlMode::PercentOutput,0);
    break;
  }
}
