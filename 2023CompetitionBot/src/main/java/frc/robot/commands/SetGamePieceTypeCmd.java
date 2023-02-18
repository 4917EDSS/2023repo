// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedZones;

public class SetGamePieceTypeCmd extends CommandBase {
  private final LedSub m_ledSub;
  private final RobotContainer m_robotContainer;
  private final boolean m_coneMode;

  /** Creates a new SetGamePieceTypeCmd. */
  public SetGamePieceTypeCmd(boolean coneMode, RobotContainer robotContainer, LedSub ledSub) {
    m_coneMode = coneMode;
    m_robotContainer = robotContainer;
    m_ledSub = ledSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotContainer.setConeMode(m_coneMode);

    if (m_coneMode) {
      m_ledSub.setZoneColour(LedZones.ZONE1, 0, 0, 128);
    } else {
      m_ledSub.setZoneColour(LedZones.ZONE1, 0, 128, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
