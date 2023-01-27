// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkMaxPIDController m_armMotorPIDController;
  private final AbsoluteEncoder m_armEncoder;

    public SUB_Arm() {
      m_armMotor = new CANSparkMax(ArmConstants.kArmMotorCanID, MotorType.kBrushless);
      m_armMotorPIDController = m_armMotor.getPIDController();
      m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_armMotorPIDController.setP(ArmConstants.kArmP);
      m_armMotorPIDController.setI(ArmConstants.kArmI);
      m_armMotorPIDController.setD(ArmConstants.kArmD);
      m_armMotorPIDController.setFF(ArmConstants.kArmF);
      m_armMotorPIDController.setFeedbackDevice(m_armEncoder);
      m_armMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_armMotorPIDController.setOutputRange(-.5, 0.5);
      m_armEncoder.setPositionConversionFactor(360);
      m_armEncoder.setInverted(true);
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
