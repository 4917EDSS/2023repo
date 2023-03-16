// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj.RobotController;


public class RotateRobotCmd extends CommandBase {
  private final DrivetrainSub m_drivetrainSub;
  private final double m_angle;
  private long m_startTime;

  private static double kMinPower = 0.2;
  private static double kMaxPower = 0.8;
  private static double kTolerance = 1;
  private double m_rotationRemaining = 0;


  /** Creates a new RotateRobotCmd. */
  public RotateRobotCmd(DrivetrainSub drivetrainSub, double angle) {
    m_drivetrainSub = drivetrainSub;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.shift(false);
    m_drivetrainSub.zeroHeading();
    m_startTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double power = 0.8;
    double rotationRemaining = 0;
    rotationRemaining = m_angle - m_drivetrainSub.getHeading();


    double dir = (rotationRemaining < 0) ? -1 : 1;
    rotationRemaining = Math.abs(rotationRemaining);
    if(rotationRemaining <= 40) {
      power = ((rotationRemaining / 40) * (kMaxPower - kMinPower)) + kMinPower;
    }
    if(rotationRemaining <= kTolerance) {
      power = 0;
    }

    m_drivetrainSub.arcadeDrive(0, power * dir);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_rotationRemaining <= kTolerance) && (Math.abs(m_drivetrainSub.getTurnRate()) <= 0.1)) {
      return true;
    }
    if((RobotController.getFPGATime() - m_startTime) > 3000000) {
      return true;
    }
    return false;
  }
}
