// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright © 2023 Riley Fisher. No rights reserved.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.ManipulatorsPositions;
import frc.robot.subsystems.LedSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickUpGrp extends SequentialCommandGroup {
  /** Creates a new AutoPickUpCmd. */
  public AutoPickUpGrp(ManipulatorsPositions positions, ArmSub armSub, MastSub mastSub, IntakeSub intakeSub,
      DrivetrainSub drivetrainSub, LedSub ledSub, double targetDriveDistance, double power) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /* \ Sets the intake position, drives in, then grabs \ */
    addCommands(new IntakeSetPositionCmd(positions, armSub, mastSub, intakeSub),
        new DriveStraightCmd(drivetrainSub, targetDriveDistance, 0.8),
        new IntakeGamePieceCmd(power, intakeSub, ledSub));
  }
}
