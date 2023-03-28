// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"

DriveCommand::DriveCommand(DriveSubsystem* driveSubsystem, double distance)
    : m_driveSubsystem{driveSubsystem},
      m_distance{distance},
      m_pidController{0,0,0} {
  // Register that this command requires the subsystem.
  AddRequirements(m_driveSubsystem);

  m_kP = 0.5;
  m_kI = 0;
  m_kD = 0;
}

void DriveCommand::Initialize() {
  m_driveSubsystem->SetMaxOutput(0.25);
  m_pidController.SetPID(m_kP,m_kI,m_kD);
  m_pidController.SetTolerance(0.01);
  m_startingPos = m_driveSubsystem->GetAverageEncoderDistance();
  m_pidController.SetSetpoint(m_startingPos+m_distance);
}

void DriveCommand::Execute() {
  double currPos = m_driveSubsystem->GetAverageEncoderDistance();
  double speed = m_pidController.Calculate(currPos);
  std::cout << "speed " << speed << std::endl;
  m_driveSubsystem->ArcadeDrive(speed,0);
}

void DriveCommand::End(bool interrupted) {
  m_driveSubsystem->ArcadeDrive(0,0);
}

bool DriveCommand::IsFinished() {
  return m_pidController.AtSetpoint();
}