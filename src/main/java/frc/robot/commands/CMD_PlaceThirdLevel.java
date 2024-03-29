// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PlaceThirdLevel extends SequentialCommandGroup {
  /** Creates a new CMD_PlaceThirdLevel. */
  SUB_Arm m_arm;
  SUB_Intake m_intake;
  public CMD_PlaceThirdLevel(SUB_Arm p_arm, SUB_Intake p_intake) {
    m_arm = p_arm;
    m_intake = p_intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_ArmSetPosition(m_arm, 171),
      new CMD_ArmCheck(m_arm, 171),
      new CMD_DropItem(m_intake),
      new WaitCommand(1),
      new CMD_Stow(m_arm, m_intake)
    );
  }
}
