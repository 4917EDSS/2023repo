// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveFwdThenBackGrp extends SequentialCommandGroup {

  /** Creates a new AutoDriveFwdThenBack. */
  public AutoDriveFwdThenBackGrp(DrivetrainSub drivetrainSub, double targetDriveDistanceFwd,
      double targetDriveDistanceBack) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveStraightCmd(drivetrainSub, targetDriveDistanceFwd),
        new DriveStraightCmd(drivetrainSub, -targetDriveDistanceBack));
  }
}
