// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import frc.robot.subsystems.SUB_FiniteStateMachine;

public class CMD_Intake extends CommandBase {
  SUB_Elevator m_elevator;
  SUB_Intake m_intake;
  SUB_FiniteStateMachine m_finiteStateMachine;
  SUB_Elbow m_elbow;
  SUB_Wrist m_wrist;

  public CMD_Intake(

    SUB_Intake p_intake,
    SUB_Elevator p_elevator,
    SUB_FiniteStateMachine p_finiteStateMachine,
    SUB_Elbow p_elbow, 
    SUB_Wrist p_wrist

    ){

    m_elevator = p_elevator;
    m_intake = p_intake;
    m_finiteStateMachine = p_finiteStateMachine;
    m_elbow = p_elbow;
    m_wrist = p_wrist;
  }

  @Override
  public void initialize() {
    m_finiteStateMachine.setState(RobotState.INTAKING);
    m_elevator.setPosition(173);
    m_wrist.setReference(215);
    m_elbow.setReference(180);
    
    m_intake.setIntakeCurrent();
    if(m_intake.getIntakeState() == true){
      m_intake.setIntakeForward();
    }else{
      m_intake.setIntakeReverse();
    }
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