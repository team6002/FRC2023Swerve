// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;


public class SUB_LimeLight extends SubsystemBase {
  SUB_Blinkin m_blinkin;
  SUB_FiniteStateMachine m_finiteStateMachine;
  public SUB_LimeLight(SUB_Blinkin p_blinkin, SUB_FiniteStateMachine p_finiteStateMachine) {
    m_blinkin = p_blinkin;
    m_finiteStateMachine = p_finiteStateMachine;
  }

  public CommandBase exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("DO YOU SEE ANYTHING ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
  
  if(hasTarget() && m_finiteStateMachine.getState() == RobotState.SCORING){
    m_blinkin.setHasTarget();
  }
  else{
    m_blinkin.setHasNoTarget();
  }
  }

  @Override
  public void simulationPeriodic() {
  }

  public boolean hasTarget(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
      return true;
    }
    else{
      return false;
    }
  }
}