// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PlaceSecondLevel extends SequentialCommandGroup {
  /** Creates a new CMD_PlaceThirdLevel. */
  SUB_Elevator m_elevator;
  SUB_Intake m_intake;
  public CMD_PlaceSecondLevel(SUB_Elevator p_elevator, SUB_Intake p_intake) {
    m_elevator = p_elevator;
    m_intake = p_intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_ElevatorSetPosition(m_elevator, 164),
      new CMD_ElevatorCheck(m_elevator, 164),
      new CMD_DropItem(m_intake),
      new WaitCommand(1),
      new CMD_Stow(m_elevator, m_intake)
    );
  }
}
