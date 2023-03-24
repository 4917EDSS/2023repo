// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveStraightCmd extends CommandBase {
  private final DrivetrainSub m_drivetrainSub;
  private double m_distanceRemaining = 0;
  private double kRotateAdjustment = 0.045;
  private double kMaxPower = 0.9;
  private double kMinPower = 0.2;
  private double kTolerance = 0.03;
  private double m_targetDriveDistance;
  private long m_timeStart;

  /** Creates a new DriveStraightCmd. */
  public DriveStraightCmd(DrivetrainSub drivetrainSub, double targetDriveDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSub = drivetrainSub;
    m_targetDriveDistance = targetDriveDistance;
    addRequirements(drivetrainSub);
    double minimumMax = 0.4;
    double minimumDistance = 0.5;
    double fullSpeed = 0.9;
    double fullDistance = 3;

    if(targetDriveDistance < minimumDistance) {
      kMaxPower = minimumMax;
    } else if(targetDriveDistance > fullDistance) {
      kMaxPower = fullSpeed;
    } else {
      kMaxPower =
          0.4 + ((fullSpeed - minimumMax) / (fullDistance - minimumDistance)) * (targetDriveDistance - minimumDistance);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.zeroHeading();
    m_drivetrainSub.zeroDrivetrainEncoders();
    m_timeStart = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotatePwr = m_drivetrainSub.getHeading() * kRotateAdjustment;
    double power = kMaxPower;
    m_distanceRemaining = Math.abs(m_targetDriveDistance) - Math.abs(m_drivetrainSub.getEncoderDistanceM());
    double dir = (m_distanceRemaining < 0) ? -1 : 1;
    m_distanceRemaining = Math.abs(m_distanceRemaining);


    if(m_distanceRemaining <= 0.3) {
      power = ((m_distanceRemaining / 0.3) * (kMaxPower)) + kMinPower;
    }
    if(m_distanceRemaining <= kTolerance) {
      power = 0;
    }

    m_drivetrainSub.arcadeDrive(power * dir * (m_targetDriveDistance / Math.abs(m_targetDriveDistance)), -rotatePwr);
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
    if(RobotController.getFPGATime() - m_timeStart > (2500000 * m_targetDriveDistance)) { // After 2.5 seconds the command stops automatically
      System.out.println("**** timed out** intake set position");
      return true;
    }
    return false;
  }
}
