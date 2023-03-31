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
//   if (m_stage == 1){
//     kP = 0.02;
//   } else if (m_stage == 2) {
//     kP = 0.01;
//   }
  kP = 0.015;
  kI = 0;
  kD = 0;
}

void AutoBalanceCommand::Initialize() {
    m_driveSubsystem->SetDriveSpeedModifier(1);
    std::cout << "beginning to balance" << std::endl;
    m_driveSubsystem->SetMaxOutput(0.25);
    m_driveSubsystem->SetNeutralMode(NeutralMode::Brake);
    m_balancePIDController.SetPID(kP,kI,kD);
    m_balancePIDController.SetSetpoint(0);
    if (m_stage == 1)
    {
        m_balancePIDController.SetTolerance(4);
        isCommandFinished = false;
    }
    else if(m_stage == 2)
    {
        m_balancePIDController.SetTolerance(1);
        isCommandFinished = false;
    } else if(m_stage == 0)
    {
        isCommandFinished = true;
    }
    
}

void AutoBalanceCommand::Execute() {
    double currAngle = m_driveSubsystem->GetAngle();
    double motorSpeed = m_balancePIDController.Calculate(currAngle);
    frc::SmartDashboard::PutNumber("Current Error",m_balancePIDController.GetPositionError());
    frc::SmartDashboard::PutNumber("Balance Speed",motorSpeed);
    if (m_stage == 1)
    {
        m_driveSubsystem->ArcadeDrive(motorSpeed*0.75,0);
    }
    else if(m_stage == 2)
    {
        if(motorSpeed>0) {
            motorSpeed += 0.02;
        } else if (motorSpeed < 0)
        {
            motorSpeed -= 0.02 ;
        }
        
        m_driveSubsystem->ArcadeDrive(motorSpeed*0.6,0);
    }
}

bool AutoBalanceCommand::IsFinished() {
    if (isCommandFinished == true || m_balancePIDController.AtSetpoint()) {
        m_driveSubsystem->SetDriveSpeedModifier(0.75);
        std::cout<<"is finished"<< std::endl;  
        return true;
    } else
    {
        std::cout<<"is finished"<< std::endl;  
        return m_balancePIDController.AtSetpoint();
    }
}