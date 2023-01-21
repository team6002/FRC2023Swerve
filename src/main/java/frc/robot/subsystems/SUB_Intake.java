// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.BlinkinConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final SparkMaxPIDController m_intakeMotorPIDController;
    private final DigitalInput m_sensor;
    private final SUB_FiniteStateMachine m_finiteStateMachine;
    private final SUB_Blinkin m_blinkin;
  /** Creates a new SUB_Intake. */
  public SUB_Intake(SUB_FiniteStateMachine p_finiteStateMachine, SUB_Blinkin p_blinkin) {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    m_intakeMotorPIDController = m_intakeMotor.getPIDController();
    m_sensor = new DigitalInput(1);
    m_finiteStateMachine = p_finiteStateMachine;
    m_blinkin = p_blinkin;
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

  public boolean getSensor(){
    return m_sensor.get();
  }

  @Override
  public void periodic(){
    // if we have a game piece, make the led strip sky blue colored
    if(m_finiteStateMachine.getState() == RobotState.INTAKING && getSensor()){
      m_blinkin.set(BlinkinConstants.kSkyBlue);
    }
  }
}
