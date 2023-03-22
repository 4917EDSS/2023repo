// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ManipulatorsPositions;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSetPositionDelayCmd extends SequentialCommandGroup {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;

  /** Creates a new IntakeSetPositionDelayCmd. */
  public IntakeSetPositionDelayCmd(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub) {
    m_armSub = armSub;
    m_mastSub = mastSub;
    m_intakeSub = intakeSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitCommand(0.5),
        new IntakeSetPositionCmd(ManipulatorsPositions.HOME, m_armSub, m_mastSub, m_intakeSub));
  }
}
