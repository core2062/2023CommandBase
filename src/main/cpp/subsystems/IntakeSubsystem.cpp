// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

using namespace DriveConstants;

IntakeSubsystem::IntakeSubsystem() : m_intakeMotor{kIntakeMotorPort} {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr IntakeSubsystem::IntakeMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void IntakeSubsystem::SetIntakeMotor(double speed){
    m_intakeMotor.Set(ControlMode::PercentOutput,speed);
}
