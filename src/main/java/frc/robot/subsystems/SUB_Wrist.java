// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Wrist extends SubsystemBase {

    private final CANSparkMax m_wristMotor;
    private final SparkMaxPIDController m_wristMotorPIDController;
    private final AbsoluteEncoder m_wristEncoder;

    public SUB_Wrist() {
        m_wristMotor = new CANSparkMax(WristConstants.kWristMotorCanID, MotorType.kBrushless);
        m_wristMotorPIDController = m_wristMotor.getPIDController();
        m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristEncoder.setPositionConversionFactor(360);
        m_wristEncoder.setVelocityConversionFactor(6);
        m_wristEncoder.setInverted(true);
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry(){
        SmartDashboard.putNumber("position", m_wristEncoder.getPosition());
    }

    public void setWristReference(double p_reference){
        m_wristMotorPIDController.setReference(p_reference, ControlType.kPosition);
    }

    public double getWristPosition(){
        return m_wristEncoder.getPosition();
    }
}
