// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class DriveAlignToVisionCmd extends CommandBase {
  /** Creates a new AlignToVisionCmd. */

  private final DrivetrainSub m_drivetrainSub;
  private final VisionSub m_visionSub;
  private final PIDController m_angleController = new PIDController(0.1, 0.0, 0.01); // Using a PID controller instead of a function
  private final double kMaxPower = 0.45; // Maximum possible motor power
  private double kPower = 0.0;
  private long m_timeStart; // Time constraint incase it never aligns

  public DriveAlignToVisionCmd(DrivetrainSub drivetrain, VisionSub vision) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrainSub = drivetrain;
    m_visionSub = vision;

    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeStart = RobotController.getFPGATime();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle_offset = m_visionSub.getHorizontalAngle();
    kPower = MathUtil.clamp(m_angleController.calculate(-angle_offset, 0.0), -1.0, 1.0);
    m_drivetrainSub.arcadeDrive(0.0, kPower * kMaxPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_visionSub.getTargetArea() == 0.0) {
      return true;
    }
    if(RobotController.getFPGATime() - m_timeStart > 3000000) { // After 3 seconds the command stops automatically
      return true;
    }

    if(Math.abs(kPower) < 0.01) {
      return true;
    }

    return false;
  }
}
