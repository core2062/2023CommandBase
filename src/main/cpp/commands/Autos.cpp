// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RamseteCommand.h>

#include "commands/ExampleCommand.h"
#include "commands/AutoBalanceCommand.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

// frc2::CommandPtr autos::AutoBalanceAuto(DriveSubsystem* m_driveSubsystem) {
//   frc2::RamseteCommand* ramseteCommand = &RobotContainer::GetMoveBackCommand()

//   return frc2::cmd::Sequence();
// }
