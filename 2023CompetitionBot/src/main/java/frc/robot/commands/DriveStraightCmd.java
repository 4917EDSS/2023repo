// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveStraightCmd extends CommandBase {

  private final DrivetrainSub m_drivetrainSub;
  private double m_distanceRemaining = 0;
  private double kRotateAdjustment = 0.045;
  private double kMinPower = 0.15;
  private double kTolerance = 0.03;
  private double m_targetDriveDistance;

  /** Creates a new DriveStraightCmd. */
  public DriveStraightCmd(DrivetrainSub drivetrainSub, double targetDriveDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSub = drivetrainSub;
    m_targetDriveDistance = targetDriveDistance;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotatePwr = 1;
    double power = 1;
    m_distanceRemaining = m_targetDriveDistance - m_drivetrainSub.getEncoderDistanceM();
    m_distanceRemaining = Math.abs(m_distanceRemaining);
    double dir = (m_distanceRemaining < 0) ? -1 : 1;

    if(m_distanceRemaining <= 0.4) {
      power = (m_distanceRemaining / 0.4) * (1 - kMinPower) + kMinPower;
    }
    if(m_distanceRemaining <= kTolerance) {
      power = 0;
    }

    m_drivetrainSub.arcadeDrive(power * dir, -rotatePwr);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_distanceRemaining <= kTolerance) && (Math.abs(m_drivetrainSub.getVelocity()) <= 0.1)) {
      return true;
    }
    return false;
  }
}
