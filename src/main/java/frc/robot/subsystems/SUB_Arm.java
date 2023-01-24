// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkMaxPIDController m_armMotorPIDController;
  private final RelativeEncoder m_armEncoder;

    public SUB_Arm() {
      m_armMotor = new CANSparkMax(ArmConstants.kArmMotorCanID, MotorType.kBrushless);
      m_armMotorPIDController = m_armMotor.getPIDController();
      m_armEncoder = m_armMotor.getEncoder();
  }

  public void setPosition(double position){
    m_armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setArmOn(){
    m_armMotor.set(ArmConstants.kArmForward);
  }

  public void setArmOff(){
    m_armMotor.set(0);
  }

  public void setArmReverse(){
    m_armMotor.set(-ArmConstants.kArmForward);
  }

  @Override
  public void periodic() {
    // updates arm telemetry
    telemetry();
  }

  public void telemetry(){
    SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
  }
}
