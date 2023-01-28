// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;

public class CMD_HoldCube extends CommandBase {
  SUB_Arm m_arm;
  SUB_Intake m_intake;
  public CMD_HoldCube(SUB_Intake p_intake, SUB_Arm p_arm) {
    m_arm = p_arm;
    m_intake = p_intake;
  }

  @Override
  public void initialize() {
    m_arm.setPosition(15);
    m_intake.setHoldCurrent();
    m_intake.setPower(-0.07);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
