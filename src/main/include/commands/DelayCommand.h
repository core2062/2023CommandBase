// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DelayCommand
    : public frc2::CommandHelper<frc2::CommandBase, DelayCommand> {
 public:
  /**
   * Creates a new DelayCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit DelayCommand(DriveSubsystem* drive,units::time::second_t time);

  void Initialize() override;

  void Execute() override;
  
  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  units::time::second_t m_time;
  frc::Timer m_timer;
};
