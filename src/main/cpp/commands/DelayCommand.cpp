// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DelayCommand.h"

DelayCommand::DelayCommand(DriveSubsystem* drive, units::time::second_t time)
    : m_time{time},
      m_drive{drive} {
  // Register that this command requires the subsystem.
}

void DelayCommand::Initialize() {
  m_timer.Restart();
}

void DelayCommand::Execute() {
  m_drive->TankDriveVolts(0_V, 0_V);
}

bool DelayCommand::IsFinished() {
  return m_timer.HasElapsed(m_time);
}