// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;

public class CMD_ThirdLevel extends CommandBase {
  /** Creates a new CMD_ThirdLevel. */
  SUB_Arm m_arm;
  SUB_Intake m_intake;
  double m_timer = 1;

  public CMD_ThirdLevel(SUB_Arm p_arm, SUB_Intake p_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_intake = p_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setPosition(164);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( m_arm.getPosition() >= 161 && m_arm.getPosition() <= 167 ){
      if (m_intake.getIntakeState() == true){
        m_intake.setIntakeReverse();
      } else {
        m_intake.setIntakeForward();
      }
    
      if (m_timer == 10 ){
        m_timer =10;
      } else {
        m_timer += 1;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.getPosition() == 164 && m_timer == 10){
      
      return true;
    }else return false;
    
  }
}
