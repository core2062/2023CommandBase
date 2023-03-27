// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeSpeedCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeSpeedCommand> {
 public:
  /**
   * Creates a new IntakeSpeedCommand.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  explicit IntakeSpeedCommand(IntakeSubsystem* intakeSubsystem, double intakeSpeed);
  
  void Execute() override;

 private:
  IntakeSubsystem* m_intakeSubsystem;
  double m_intakeSpeed;
};
