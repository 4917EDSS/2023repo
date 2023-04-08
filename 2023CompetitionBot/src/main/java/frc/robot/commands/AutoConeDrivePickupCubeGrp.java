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
import frc.robot.subsystems.ManipulatorsPositions;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeDrivePickupCubeGrp extends SequentialCommandGroup {


  /** Creates a new AutoConeAndLeave. */

  public AutoConeDrivePickupCubeGrp(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub,
      DrivetrainSub drivetrainSub, LedSub ledSub, boolean isClose) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoConeSubGrp(armSub, mastSub, intakeSub, drivetrainSub, ledSub),
        new DriveStraightCmd(drivetrainSub, 4.2, 0.8)
            .alongWith(new IntakeSetPositionDelayGrp(armSub, mastSub, intakeSub)),
        new RotateRobotCmd(drivetrainSub, -173, true, isClose),
        new SetGamePieceTypeCmd(false, ledSub),
        new IntakeSetPositionCmd(ManipulatorsPositions.TILTED_GROUND_CUBE, armSub, mastSub,
            intakeSub),
        new IntakeGamePieceCmd(1.0, intakeSub, ledSub)
            .alongWith(new DriveStraightCmd(drivetrainSub, -1.1, 0.8)),
        new IntakeSetPositionCmd(ManipulatorsPositions.HOME, armSub, mastSub, intakeSub),
        new RotateRobotCmd(drivetrainSub, 168, true, isClose),
        new DriveStraightCmd(drivetrainSub, -0.5, 0.8));
  }
}
