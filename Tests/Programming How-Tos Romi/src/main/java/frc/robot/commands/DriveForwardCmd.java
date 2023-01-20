// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveForwardCmd extends CommandBase {
  // Need a member variable to save the DrivetrainSub that was passed into the command
  private final DrivetrainSub m_drivetrainSub;

  /** Creates a new DriveForwardCmd. */
  public DriveForwardCmd(DrivetrainSub drivetrainSub) {
    // This is the constructor method.  It runs when the command gets created.
    // Save the subsystem that was passed in for later use
    m_drivetrainSub = drivetrainSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set both sides of the robot to 25% power
    m_drivetrainSub.tankDrive(0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors
    m_drivetrainSub.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
