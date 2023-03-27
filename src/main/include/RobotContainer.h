// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc/Compressor.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/IntakeSpeedCommand.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::GenericHID m_driverController{OIConstants::kDriverControllerPort};
  frc::GenericHID m_operatorController{OIConstants::kOperatorControllerPort};
  // frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;

  frc2::InstantCommand m_driveHalfSpeed{[this] { m_drive.SetMaxOutput(0.5); },
                                        {}};
  frc2::InstantCommand m_driveFullSpeed{[this] { m_drive.SetMaxOutput(1); },
                                        {}};
                                        
  frc2::StartEndCommand m_intakePositionToggle{[&] { m_intake.SetIntakeSolenoid(DoubleSolenoid::kForward); },
                                               [&] { m_intake.SetIntakeSolenoid(DoubleSolenoid::kReverse); },
                                                {&m_intake}};
  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  


  void ConfigureButtonBindings();
  
  // Compressor m_compressor;
};