// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

using namespace DriveConstants;

IntakeSubsystem::IntakeSubsystem() : m_intakeUpperMotor(kIntakeUpperMotorPort),
                                     m_intakeLowerMotor(kIntakeLowerMotorPort),
                                     m_intakeSolenoid(PneumaticsModuleType::REVPH,kIntakeSolenoidIn,kIntakeSolenoidOut) {
  // Implementation of subsystem constructor goes here.
  m_intakeSpeed = 0;

  m_intakeLowerMotor.SetInverted(true);
  m_intakeLowerMotor.Follow(m_intakeUpperMotor);

  m_intakeUpperMotor.Set(ControlMode::PercentOutput,0);
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

void IntakeSubsystem::SetIntakeMotors(double speed){
  m_intakeUpperMotor.Set(ControlMode::PercentOutput,speed);  
}

void IntakeSubsystem::SetIntakeSolenoid(DoubleSolenoid::Value value) {
  m_intakeSolenoid.Set(value);
}