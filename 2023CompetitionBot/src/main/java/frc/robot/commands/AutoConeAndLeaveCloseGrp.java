// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeAndLeaveCloseGrp extends SequentialCommandGroup {
  /** Creates a new AutoConeAndLeaveClose. */
  public AutoConeAndLeaveCloseGrp(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub,
      DrivetrainSub drivetrainSub,
      LedSub ledSub) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoConeAndReverseGrp(armSub, mastSub, intakeSub, drivetrainSub, ledSub),
        new DriveStraightCmd(drivetrainSub, 2, 0.8),
        new RotateRobotCmd(drivetrainSub, 15, true, false),
        new DriveStraightCmd(drivetrainSub, 2, 0.8));
  }
}
