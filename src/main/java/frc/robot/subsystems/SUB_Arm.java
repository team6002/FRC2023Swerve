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

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkMaxPIDController m_armMotorPIDController;
  private final AbsoluteEncoder m_armEncoder;
  private double m_wantedPosition;
  private int m_wantedSlot;
  private double arm_up_p, arm_up_i, arm_up_d, arm_up_f;
  private double arm_down_p, arm_down_i, arm_down_d, arm_down_f;

    public SUB_Arm() {
      m_armMotor = new CANSparkMax(ArmConstants.kArmMotorCanID, MotorType.kBrushless);
      m_armMotorPIDController = m_armMotor.getPIDController();
      m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_armEncoder.setPositionConversionFactor(360);
      m_armEncoder.setVelocityConversionFactor(6);
      m_armEncoder.setInverted(true);
      // m_armMotorPIDController.setP(ArmConstants.kArmP);
      // m_armMotorPIDController.setI(ArmConstants.kArmI);
      // m_armMotorPIDController.setD(ArmConstants.kArmD);
      // m_armMotorPIDController.setFF(ArmConstants.kArmF);
      m_armMotorPIDController.setP(0.0,1);
      m_armMotorPIDController.setI(0,1);
      m_armMotorPIDController.setD(0,1);
      m_armMotorPIDController.setFF(0.0,1);
      m_armMotorPIDController.setFeedbackDevice(m_armEncoder);
      m_armMotor.setIdleMode(IdleMode.kCoast);
      m_armMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_armMotorPIDController.setOutputRange(-1, 1, 1);
      m_armMotorPIDController.setSmartMotionMaxVelocity(20, 1);
      m_armMotorPIDController.setSmartMotionMaxAccel(20, 1);
      m_armMotorPIDController.setSmartMotionMinOutputVelocity(0, 1);
      m_armMotorPIDController.setSmartMotionAllowedClosedLoopError(5, 1);
      m_armMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);

      m_armMotorPIDController.setP(0.000001,2);
      m_armMotorPIDController.setI(0,2);
      m_armMotorPIDController.setD(0,2);
      m_armMotorPIDController.setFF(0.005,2);
      m_armMotorPIDController.setOutputRange(-1, 1, 2);
      m_armMotorPIDController.setSmartMotionMaxVelocity(10, 2);
      m_armMotorPIDController.setSmartMotionMinOutputVelocity(0, 2);
      m_armMotorPIDController.setSmartMotionMaxAccel(10, 2);
      m_armMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 2);
      m_armMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 2);

      m_wantedSlot = 1;
    }

    // public void setPosition(double position){
    //   m_armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    // }
    public void setPosition(double p_reference){
      m_wantedPosition = p_reference;
      if(p_reference > 100){
        m_armMotorPIDController.setP(arm_up_p,1);
        m_armMotorPIDController.setI(arm_up_i,1);
        m_armMotorPIDController.setD(arm_up_d,1);
        m_armMotorPIDController.setFF(arm_up_f,1);
        m_armMotorPIDController.setSmartMotionMaxVelocity(20, 1);
        m_armMotorPIDController.setSmartMotionMaxAccel(20, 1);
        m_wantedSlot = 1;
        m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kPosition, 1);
        // m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kSmartMotion,1);
      }else{
        m_wantedSlot = 1;
        m_armMotorPIDController.setP(arm_down_p,1);
        m_armMotorPIDController.setI(arm_down_i,1);
        m_armMotorPIDController.setD(arm_down_d,1);
        m_armMotorPIDController.setFF(arm_down_f,1);
        m_armMotorPIDController.setSmartMotionMaxVelocity(10, 1);
        m_armMotorPIDController.setSmartMotionMaxAccel(10, 1);
        m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kPosition, 1);
        // m_armMotorPIDController.setReference(p_reference, CANSparkMax.ControlType.kSmartMotion, 1);
      }
    } 
    public double getPosition(){
      return m_armEncoder.getPosition();
    }
  public void setArmOn(){
    m_armMotor.set(ArmConstants.kArmForward);
  }

  public void setArmOff(){
    m_armMotorPIDController.setReference(m_armEncoder.getPosition(), ControlType.kSmartMotion);
  }

  public void setArmReverse(){
    m_armMotor.set(-ArmConstants.kArmForward);
  }

  @Override
  public void periodic() {
    // updates arm telemetry
    telemetry();
  }
  
  // double m_P = ArmConstants.kArmP;
  // double m_I = ArmConstants.kArmI;
  // double m_D = ArmConstants.kArmD;
  // double m_F = ArmConstants.kArmF;
  public void telemetry(){
    SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Position (numeric)", m_armEncoder.getPosition());
    // SmartDashboard.putNumber("raw Arm Position", m_armEncoder.getPosition()/360);
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
   
    //   m_armMotorPIDController.setP(m_P,1);
    //   m_armMotorPIDController.setI(m_I,1);
    //   m_armMotorPIDController.setD(m_D,1);
    //   m_armMotorPIDController.setFF(m_F,1);
    //   setPosition(m_wantedPosition);
      SmartDashboard.putNumber("velcoity", m_armEncoder.getVelocity());
      SmartDashboard.putNumber("output", m_armMotor.getAppliedOutput());
      SmartDashboard.putNumber("wantedspeed", m_armMotor.get());
      // SmartDashboard.putNumber("AccelStrat", m_armMotorPIDController.getSmartMotionMaxAccel(1));
      SmartDashboard.putNumber("Arm P", m_armMotorPIDController.getP());
      SmartDashboard.putNumber("Arm I", m_armMotorPIDController.getI());
      SmartDashboard.putNumber("Arm D", m_armMotorPIDController.getD());

      SmartDashboard.putNumber("Arm F", m_armMotorPIDController.getFF());
      SmartDashboard.putNumber("ArmSetpoint", m_wantedPosition);

      SmartDashboard.putNumber("PID Slot", m_wantedSlot);
      SmartDashboard.putNumber("Slot P=", m_armMotorPIDController.getP(m_wantedSlot));
      SmartDashboard.putNumber("Slot I=", m_armMotorPIDController.getI(m_wantedSlot));
      SmartDashboard.putNumber("Slot D=", m_armMotorPIDController.getD(m_wantedSlot));
      SmartDashboard.putNumber("Slot F=", m_armMotorPIDController.getFF(m_wantedSlot));
      SmartDashboard.putNumber("Arm RampRate(s)", m_armMotor.getClosedLoopRampRate());

      arm_up_p = SmartDashboard.getNumber("Arm UP P=", 0);
      arm_up_i = SmartDashboard.getNumber("Arm UP I=", 0);
      arm_up_d = SmartDashboard.getNumber("Arm UP D=", 0);
      arm_up_f = SmartDashboard.getNumber("Arm UP F=", 0);

      SmartDashboard.putNumber("Arm UP P=", arm_up_p);
      SmartDashboard.putNumber("Arm UP I=", arm_up_i);
      SmartDashboard.putNumber("Arm UP D=", arm_up_d);
      SmartDashboard.putNumber("Arm UP F=", arm_up_f);

      arm_down_p = SmartDashboard.getNumber("Arm DOWN P=", 0);
      arm_down_i = SmartDashboard.getNumber("Arm DOWN I=", 0);
      arm_down_d = SmartDashboard.getNumber("Arm DOWN D=", 0);
      arm_down_f = SmartDashboard.getNumber("Arm DOWN F=", 0);

      SmartDashboard.putNumber("Arm DOWN P=", arm_down_p);
      SmartDashboard.putNumber("Arm DOWN I=", arm_down_i);
      SmartDashboard.putNumber("Arm DOWN D=", arm_down_d);
      SmartDashboard.putNumber("Arm DOWN F=", arm_down_f);
  }
}
