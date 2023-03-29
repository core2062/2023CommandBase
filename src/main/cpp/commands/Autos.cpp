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

//   return new frc2::cmd::Sequence(std::move(&RobotContainer::GetRamseteCommand("BlueMoveBack.wpilib.json")),
//       frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
//       std::move(AutoBalanceCommand{&m_drive,1}),
//       std::move(DelayCommand(&m_drive,1.0_s)),
//       std::move(AutoBalanceCommand{&m_drive,2}),
//       frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
// }
