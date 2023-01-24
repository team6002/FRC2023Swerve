// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LimeLight;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Arm m_arm = new SUB_Arm();
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final SUB_LimeLight m_limeLight = new SUB_LimeLight(m_blinkin, m_finiteStateMachine);
  private final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(m_blinkin, m_finiteStateMachine, m_limeLight);
  private final SUB_Intake m_intake = new SUB_Intake(m_finiteStateMachine, m_blinkin);
  // The driver's controller
  XboxController m_operatorController = new XboxController(1);
  CommandXboxController m_driverControllerTrigger = new CommandXboxController(0);
  Trigger xButton = m_driverControllerTrigger.leftBumper();

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new CMD_DriveCommand(m_robotDrive, m_driverControllerTrigger));
  }
  boolean pressed = false;
  private void configureButtonBindings() {
    m_driverControllerTrigger.leftBumper().onTrue(new CMD_IntakeForward(m_intake));
    m_driverControllerTrigger.rightBumper().onTrue(new CMD_IntakeReverse(m_intake));
    m_driverControllerTrigger.b().onTrue(new CMD_IntakeOff(m_intake));
    m_driverControllerTrigger.y().whileTrue(new CMD_ArmSetOn(m_arm));
    m_driverControllerTrigger.x().whileTrue(new CMD_ArmSetReverse(m_arm));
    m_driverControllerTrigger.y().whileFalse(new CMD_ArmSetOff(m_arm));
    m_driverControllerTrigger.x().whileFalse(new CMD_ArmSetOff(m_arm));
    m_driverControllerTrigger.back().onTrue(new CMD_ArmSetOff(m_arm));
    m_driverControllerTrigger.a().onTrue(new CMD_IntakeHold(m_intake));

    // if(m_driverController.getAButtonPressed()){
    //   m_robotDrive.getWantedLength();
    //   //use wanted length to drive
    // }

    // if(m_operatorController.getYButtonPressed()){
    //   m_robotDrive.setWantedHeight(3);
    // }

    // if(m_operatorController.getXButtonPressed()){
    //   m_robotDrive.setWantedHeight(2);
    // }

    // if(m_operatorController.getAButtonPressed()){
    //   m_robotDrive.setWantedHeight(3);
    // }
  }

  // public void runXTest() {
  //   if(m_driverController.getXButtonPressed()){
  //     // new CMD_ArmSetOff(m_arm);
  //     m_robotDrive.setX();
  //   }
  // }

    public void zeroGyroHeading() {
      m_robotDrive.zeroHeading();
    }

}