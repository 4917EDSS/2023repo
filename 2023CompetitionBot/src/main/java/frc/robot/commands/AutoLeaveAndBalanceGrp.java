// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSub;

public class AutoLeaveAndBalanceGrp extends SequentialCommandGroup {
  DrivetrainSub m_drivetrainSub;

  /** Creates a new AutoLeaveAndBalanceCmd. */
  public AutoLeaveAndBalanceGrp(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new AutoDriveOverChargeStationCmd(drivetrainSub, false),
        new AutoBalanceChargeStationCmd(drivetrainSub, true));
  }

}
