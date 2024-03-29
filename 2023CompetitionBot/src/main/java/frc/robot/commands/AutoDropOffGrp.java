// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.ManipulatorsPositions;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDropOffGrp extends SequentialCommandGroup {
  /** Creates a new AutoDropOffCmd. */
  public AutoDropOffGrp(ManipulatorsPositions positions, ArmSub armSub, MastSub mastSub, IntakeSub intakeSub,
      DrivetrainSub drivetrainSub, double targetDriveDistance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /* Sets the intake position, drives in, then drops off the game piece */
    addCommands(new IntakeSetPositionCmd(positions, armSub, mastSub, intakeSub),
        new DriveStraightCmd(drivetrainSub, targetDriveDistance, 0.8), new ExpelGamePieceCmd(1.0, intakeSub));
  }
}
