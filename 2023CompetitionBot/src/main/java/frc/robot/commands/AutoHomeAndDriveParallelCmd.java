// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.junit.rules.Timeout;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ManipulatorsPositions;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHomeAndDriveParallelCmd extends ParallelCommandGroup {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;
  private final DrivetrainSub m_drivetrainSub;

  /** Creates a new AutoHomeAndDriveParallelCmd. */
  public AutoHomeAndDriveParallelCmd(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub, DrivetrainSub drivetrainSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_armSub = armSub;
    m_mastSub = mastSub;
    m_intakeSub = intakeSub;
    m_drivetrainSub = drivetrainSub;

    addCommands(
        new DriveStraightCmd(m_drivetrainSub, 0.4),
        new ExpelGamePieceCmd(0.5, m_intakeSub)
            .andThen(new IntakeSetPositionCmd(ManipulatorsPositions.HOME, m_armSub, m_mastSub, m_intakeSub)));
  }
}
