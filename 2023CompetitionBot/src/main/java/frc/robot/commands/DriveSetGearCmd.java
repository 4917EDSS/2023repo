// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveSetGearCmd extends CommandBase {
  /** Creates a new DriveSetGearCmd. */
  private final boolean m_isHighGear;
  private final DrivetrainSub m_drivetrainSub;


  public DriveSetGearCmd(boolean isHighGear, DrivetrainSub drivetrainSub) {
    m_isHighGear = isHighGear;
    m_drivetrainSub = drivetrainSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.setIsAutoShift(false);
    m_drivetrainSub.shift(m_isHighGear);
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
