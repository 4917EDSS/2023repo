// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateOfRobot;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

public class SetGamePieceTypeCmd extends CommandBase {
  private final LedSub m_ledSub;
  private final boolean m_coneMode;

  /** Creates a new SetGamePieceTypeCmd. */
  public SetGamePieceTypeCmd(boolean coneMode, LedSub ledSub) {
    m_coneMode = coneMode;
    m_ledSub = ledSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StateOfRobot.m_coneMode = m_coneMode;

    if(m_coneMode) {
      m_ledSub.setZoneColour(LedZones.ZONE1, LedColour.BLUE);
    } else {
      m_ledSub.setZoneColour(LedZones.ZONE1, LedColour.GREEN);
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
    return true;
  }
}
