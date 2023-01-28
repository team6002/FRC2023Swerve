// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeCone extends CommandBase {
  SUB_Arm m_arm;
  SUB_Intake m_intake;
  public CMD_IntakeCone(SUB_Arm p_arm, SUB_Intake p_intake) {
    m_arm = p_arm;
    m_intake = p_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeState(false);
    m_intake.setIntakeForward();
    m_intake.setIntakeState(true);
    m_arm.setPosition(164);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}