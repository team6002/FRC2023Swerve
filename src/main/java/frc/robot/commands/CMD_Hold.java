// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;

public class CMD_Hold extends CommandBase {
  SUB_Intake m_intake;
  SUB_Elevator m_elevator;
  SUB_FiniteStateMachine m_finiteStateMachine;
  SUB_Elbow m_elbow;
  SUB_Wrist m_wrist;
  public CMD_Hold(SUB_Intake p_intake,
    SUB_Elevator p_elevator,
    SUB_FiniteStateMachine p_finiteStateMachine,
    SUB_Elbow p_elbow,
    SUB_Wrist p_wrist
    ) {
    
    m_intake = p_intake;
    m_elevator = p_elevator;
    m_finiteStateMachine = p_finiteStateMachine;
    m_elbow = p_elbow;
    m_wrist = p_wrist;
  }

  @Override
  public void initialize() {
    m_elbow.setReference(215);
    m_wrist.setReference(180);
    m_finiteStateMachine.setState(RobotState.INTAKED);
    m_intake.setHoldCurrent();
    m_elevator.setPosition(90);
    if(m_intake.getIntakeState() == true){
      m_intake.setPower(0.07);
    }else{
      m_intake.setPower(-0.07);
    }
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}