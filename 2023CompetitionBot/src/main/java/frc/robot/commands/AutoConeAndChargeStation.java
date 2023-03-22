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
public class AutoConeAndChargeStation extends SequentialCommandGroup {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;
  private final DrivetrainSub m_drivetrainSub;
  private final LedSub m_ledSub;

  /** Creates a new AutoConeAndChargeStation. */
  public AutoConeAndChargeStation(ArmSub armSub, MastSub mastSub, IntakeSub intakeSub, DrivetrainSub drivetrainSub,
      LedSub ledSub) {
    m_armSub = armSub;
    m_mastSub = mastSub;
    m_intakeSub = intakeSub;
    m_drivetrainSub = drivetrainSub;
    m_ledSub = ledSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /* Score cone, drive to charge station, then balance on charge station */
    addCommands(
        new AutoConeChargingStationGrp(armSub, mastSub, intakeSub, drivetrainSub, ledSub),
        new AutoDriveOverChargeStationCmd(drivetrainSub, true)
            .alongWith(new IntakeSetPositionDelayCmd(m_armSub, m_mastSub, m_intakeSub)),
        new AutoBalanceChargeStationCmd(drivetrainSub, false));
  }
}
