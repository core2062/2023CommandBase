// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>

#include "Constants.h"

enum class IntakeDirection {OFF, IN, OUT};

using namespace frc;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr IntakeMethodCommand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void SetIntakeMotors(double speed);

  void SetIntakeSolenoid(DoubleSolenoid::Value value);
  
  void ToggleIntakeSolenoid();

  void ToggleIntakeSolenoid();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX m_intakeUpperMotor;
  WPI_TalonSRX m_intakeLowerMotor;

  DoubleSolenoid m_intakeSolenoid;

  double m_intakeSpeed;
  bool m_intakeUp;
};
