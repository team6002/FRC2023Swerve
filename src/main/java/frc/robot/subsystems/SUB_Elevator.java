// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor;
  private final SparkMaxPIDController m_elevatorMotorPIDController;
  private final AbsoluteEncoder m_elevatorEncoder;
  private double m_wantedPosition;

    public SUB_Elevator() {
      m_elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushless);
      m_elevatorMotorPIDController = m_elevatorMotor.getPIDController();
      m_elevatorEncoder = m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_elevatorEncoder.setPositionConversionFactor(360);
      m_elevatorEncoder.setVelocityConversionFactor(6);
      m_elevatorEncoder.setInverted(true);
      // m_elevatorMotorPIDController.setP(elevatorConstants.kelevatorP);
      // m_elevatorMotorPIDController.setI(elevatorConstants.kelevatorI);
      // m_elevatorMotorPIDController.setD(elevatorConstants.kelevatorD);
      // m_elevatorMotorPIDController.setFF(elevatorConstants.kelevatorF);
      m_elevatorMotorPIDController.setP(0.000001,1);
      m_elevatorMotorPIDController.setI(0,1);
      m_elevatorMotorPIDController.setD(0,1);
      m_elevatorMotorPIDController.setFF(0.04,1);
      m_elevatorMotorPIDController.setFeedbackDevice(m_elevatorEncoder);
      m_elevatorMotor.setIdleMode(IdleMode.kCoast);
      m_elevatorMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_elevatorMotorPIDController.setOutputRange(-1, 1, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxVelocity(10, 1);
      m_elevatorMotorPIDController.setSmartMotionMinOutputVelocity(0, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxAccel(10, 1);
      m_elevatorMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_elevatorMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);

      m_elevatorMotorPIDController.setP(0.000001,2);
      m_elevatorMotorPIDController.setI(0,2);
      m_elevatorMotorPIDController.setD(0,2);
      m_elevatorMotorPIDController.setFF(0.005,2);
      m_elevatorMotorPIDController.setOutputRange(-1, 1, 2);
      m_elevatorMotorPIDController.setSmartMotionMaxVelocity(10, 2);
      m_elevatorMotorPIDController.setSmartMotionMinOutputVelocity(0, 2);
      m_elevatorMotorPIDController.setSmartMotionMaxAccel(10, 2);
      m_elevatorMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 2);
      m_elevatorMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 2);
    }

    // public void setPosition(double position){
    //   m_elevatorMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    // }
    public void setPosition(double p_reference){
      m_wantedPosition = p_reference;
      if(p_reference > 100){
        m_elevatorMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kSmartMotion,1);
      }else{
        m_elevatorMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kSmartMotion,2);
      }
    } 
    public double getPosition(){
      return m_elevatorEncoder.getPosition();
    }
  public void setElevatorOn(){
    m_elevatorMotor.set(ElevatorConstants.kElevatorForward);
  }

  public void setElevatorOff(){
    m_elevatorMotorPIDController.setReference(m_elevatorEncoder.getPosition(), ControlType.kSmartMotion);
  }

  public void setElevatorReverse(){
    m_elevatorMotor.set(-ElevatorConstants.kElevatorForward);
  }

  @Override
  public void periodic() {
    // updates elevator telemetry
    telemetry();
  }
  
  // double m_P = elevatorConstants.kelevatorP;
  // double m_I = elevatorConstants.kelevatorI;
  // double m_D = elevatorConstants.kelevatorD;
  // double m_F = elevatorConstants.kelevatorF;
  public void telemetry(){
    SmartDashboard.putNumber("elevator Position", m_elevatorEncoder.getPosition());
    SmartDashboard.putNumber("elevator Position (numeric)", m_elevatorEncoder.getPosition());
    // SmartDashboard.putNumber("raw elevator Position", m_elevatorEncoder.getPosition()/360);
    // if (SmartDashboard.getNumber("P", m_P) != m_P){
    //   m_P = SmartDashboard.getNumber("P", m_P);
    //   SmartDashboard.putNumber("P", m_P);
    // }else
    // SmartDashboard.putNumber("P", m_P);

    // if (SmartDashboard.getNumber("D", m_D) != m_D){
    //   m_D = SmartDashboard.getNumber("D", m_D);
    //   SmartDashboard.putNumber("D", m_D);
    // }else 
    // SmartDashboard.putNumber("D", m_D);

    // if (SmartDashboard.getNumber("F", m_F) != m_F){
    //   m_F = SmartDashboard.getNumber("F", m_F);
    //   SmartDashboard.putNumber("F", m_F);
    // }else
    // SmartDashboard.putNumber("F", m_F);

    // if (SmartDashboard.getNumber("wantedPosition", m_wantedPosition) != m_wantedPosition){
    //   m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);
    //   SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
    // }else
    // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
   
    //   m_elevatorMotorPIDController.setP(m_P,1);
    //   m_elevatorMotorPIDController.setI(m_I,1);
    //   m_elevatorMotorPIDController.setD(m_D,1);
    //   m_elevatorMotorPIDController.setFF(m_F,1);
    //   setPosition(m_wantedPosition);
      SmartDashboard.putNumber("velcoity", m_elevatorEncoder.getVelocity());
      SmartDashboard.putNumber("output", m_elevatorMotor.getAppliedOutput());
      SmartDashboard.putNumber("wantedspeed", m_elevatorMotor.get());
      SmartDashboard.putNumber("AccelStrat", m_elevatorMotorPIDController.getSmartMotionMaxAccel(1));
      SmartDashboard.putNumber("elevator P", m_elevatorMotorPIDController.getP());
      SmartDashboard.putNumber("elevator I", m_elevatorMotorPIDController.getI());
      SmartDashboard.putNumber("elevator D", m_elevatorMotorPIDController.getD());
      SmartDashboard.putNumber("elevator F", m_elevatorMotorPIDController.getFF());
      SmartDashboard.putNumber("elevatorSetpoint", m_wantedPosition);
  }
}
