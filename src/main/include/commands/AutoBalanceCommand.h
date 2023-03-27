// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoBalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
  /**
   * Creates a new AutoBalanceCommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  explicit AutoBalanceCommand(DriveSubsystem* driveSubsystem,int stage);

  void Initialize() override; 

  void Execute() override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_driveSubsystem;
  int m_stage;
  frc::PIDController m_balancePIDController;

  double kP, kI, kD;

};
