// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ManipulatorsPositions;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeGrp extends SequentialCommandGroup {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;

  /** Creates a new AutoConeGrp. */
  public AutoConeGrp(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub) {
    m_armSub = armSub;
    m_mastSub = mastSub;
    m_intakeSub = intakeSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeSetPositionCmd(ManipulatorsPositions.HIGH_CONE, m_armSub, m_mastSub, m_intakeSub),
        new ExpelGamePieceCmd(0.5, m_intakeSub),
        new IntakeSetPositionCmd(ManipulatorsPositions.HOME, m_armSub, m_mastSub, m_intakeSub));
  }
}
