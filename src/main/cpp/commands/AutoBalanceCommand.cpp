// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalanceCommand.h"

AutoBalanceCommand::AutoBalanceCommand(DriveSubsystem* driveSubsystem, int stage)
    : m_driveSubsystem{driveSubsystem},
      m_stage{stage},
      m_balancePIDController{0,0,0} {
  // Register that this command requires the subsystem.
  AddRequirements(m_driveSubsystem);
  if (m_stage == 1){
    kP = 0.02;
  } else if (m_stage == 2) {
    kP = 0.01;
  }
//   kP = 0.02;
//   kI = 0;
//   kD = 0;
}

void AutoBalanceCommand::Initialize() {
    m_driveSubsystem->SetNeutralMode(NeutralMode::Brake);
    m_balancePIDController.SetPID(kP,kI,kD);
    m_balancePIDController.SetSetpoint(0);
    if (m_stage == 1)
    {
        m_balancePIDController.SetTolerance(5);
    }
    else if(m_stage == 2)
    {
        m_balancePIDController.SetTolerance(1);
    }
    
}

void AutoBalanceCommand::Execute() {
    double currAngle = m_driveSubsystem->GetAngle();
    double motorSpeed = m_balancePIDController.Calculate(currAngle);
    frc::SmartDashboard::PutNumber("Current Error",m_balancePIDController.GetPositionError());
    frc::SmartDashboard::PutNumber("Balance Speed",motorSpeed);
    // std::cout << "SECOND STAGE!!!!!!! also the motor speed is: " << motorSpeed+0.1 << std::endl;
    if (m_stage == 1)
    {
        m_driveSubsystem->ArcadeDrive(motorSpeed,0);
    }
    else if(m_stage == 2)
    {
        if(motorSpeed>0) {
            motorSpeed += 0.04;
        } else if (motorSpeed < 0)
        {
            motorSpeed -= 0.04;
        }
        
        m_driveSubsystem->ArcadeDrive(motorSpeed,0);
    }
}

bool AutoBalanceCommand::IsFinished() {
    return m_balancePIDController.AtSetpoint();
}