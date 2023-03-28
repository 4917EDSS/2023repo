// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ManipulatorsPositions;
import frc.robot.subsystems.MastSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeSubGrp extends SequentialCommandGroup {
  private double m_clearanceDistance = 0.42;

  /** Creates a new AutoConeGrp. */
  public AutoConeSubGrp(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub, DrivetrainSub drivetrainSub,
      LedSub ledSub) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetGamePieceTypeCmd(true, ledSub),
        new SetLimitSwitchesCmd(mastSub, armSub, intakeSub),
        new IntakeSetPositionCmd(ManipulatorsPositions.HIGH_CONE, armSub, mastSub, intakeSub),
        new WaitCommand(0.25),
        new DriveStraightCmd(drivetrainSub, (-m_clearanceDistance)),
        new ExpelGamePieceCmd(0.5, intakeSub));
  }
}
