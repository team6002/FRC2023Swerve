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
  private double[] dv = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public double[] botpose;
  public double wanted_x;
  public double wanted_y;
  public double wanted_z;
  public double wanted_pitch;

  public CommandBase exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("DO YOU SEE ANYTHING ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
  botpose =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(dv);
  if(m_finiteStateMachine.getState() == RobotState.SCORING){  
    if(hasTarget()){
      m_blinkin.setHasTarget();
    }
    else{
      m_blinkin.setHasNoTarget();
    }
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
  public double getTargetID(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  }

  public double getTargetX(){
    return botpose[0];
  }

  public double getTargetY(){
    return botpose[1];
  }

  public double getTargetZ(){
    return botpose[2];
  }

  public double getTargetPitch(){
    return botpose[3];
  }

  public double getWantedX(){
    return wanted_x;
  }

  public double getWantedY(){
    return wanted_y;
  }
  
  public double getWantedZ(){
    return wanted_z;
  }
  public double getWantedPitch(){
    return wanted_pitch;
  }

  public void setWantedX(double p_wanted_x){
     wanted_x = p_wanted_x;
  }

  public void setWantedY(double p_wanted_y){
    wanted_y = p_wanted_y;
  }
  
  public void setWantedZ(double p_wanted_z){
    wanted_z = p_wanted_z;
  }
  public void setWantedPitch(double p_wanted_pitch){
    wanted_pitch = p_wanted_pitch;
  }
}