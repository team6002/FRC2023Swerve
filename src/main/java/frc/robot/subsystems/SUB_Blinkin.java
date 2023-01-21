// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.BlinkinConstants;

public class SUB_Blinkin extends SubsystemBase {
  Spark m_blinkin;
  // PWM m_blinkin;
  public SUB_Blinkin() {
    m_blinkin = new Spark(0);
    // m_blinkin = new PWM(0);
    // m_blinkin.setBounds(2.003, 1.50, 1.50, 1.50, 0.999);
    // m_blinkin.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
  }

  public void setHasTarget(){
    m_blinkin.set(BlinkinConstants.kGreen);
    // m_blinkin.setSpeed(BlinkinConstants.kGreen);
  }

  public void setHasNoTarget(){
    m_blinkin.set(BlinkinConstants.kRedStrobe);
    // m_blinkin.setSpeed(BlinkinConstants.kRedStrobe);
  }

  public void setHasNoGamePiece(){
    m_blinkin.set(BlinkinConstants.kRed);
    // m_blinkin.setSpeed(BlinkinConstants.kRed);
  }

  public void setCelebrate(){
    m_blinkin.set(BlinkinConstants.kFireLarge);
    // m_blinkin.setSpeed(BlinkinConstants.kFireLarge);
  }

  public void setFrontFar(){
    m_blinkin.set(BlinkinConstants.kTippedFrontFar);
    // m_blinkin.setSpeed(BlinkinConstants.kTippedFrontFar);
  }

  public void setFront(){
    m_blinkin.set(BlinkinConstants.kTippedFront);
    // m_blinkin.setSpeed(BlinkinConstants.kTippedFront);
  }

  public void setBackFar(){
    m_blinkin.set(BlinkinConstants.kTippedBack);
    // m_blinkin.setSpeed(BlinkinConstants.kTippedBack);
  }

  public void setBack(){
    m_blinkin.set(BlinkinConstants.kTippedBackFar);
    // m_blinkin.setSpeed(BlinkinConstants.kTippedBackFar);
  }

  public void set(double p_pwm){
    m_blinkin.set(p_pwm);
    // m_blinkin.setSpeed(p_pwm);
  }

  @Override
  public void periodic() {
  }
}
