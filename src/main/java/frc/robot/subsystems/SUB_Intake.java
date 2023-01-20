// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final SparkMaxPIDController m_intakeMotorPIDController;
  /** Creates a new SUB_Intake. */
  public SUB_Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    m_intakeMotorPIDController = m_intakeMotor.getPIDController();
  }

  public void setIntakeForward(){
    // m_intakeMotorPIDController.setReference(IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(.15);
  }

  public void setIntakeOff(){
    // m_intakeMotorPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(0);
  }

  public void setIntakeReverse(){
    // m_intakeMotorPIDController.setReference(-IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(-.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
