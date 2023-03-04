// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class DriveAlignTapeCmd extends CommandBase {
  /** Creates a new DriveAlignCmd. */
  private final DrivetrainSub m_drivetrainSub;
  private final VisionSub m_visionSub;
  private final PIDController m_angleController = new PIDController(0.1, 0.0, 0.01); // Using a PID controller instead of a function
  private final double kMaxPower = 0.45; // Maximum possible motor power
  private double kPower = 0.0;
  private double kOffsetAngle = 0.0;
  private double turn_dir = 1.0;
  //private long m_timeStart; // Time constraint incase it never aligns

  public DriveAlignTapeCmd(DrivetrainSub drivetrain, VisionSub vision, double offset) { // Drive towards and align to target
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrainSub = drivetrain;
    m_visionSub = vision;
    kOffsetAngle = offset;

    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turn_dir = 1.0;
    m_visionSub.setPipeline(Constants.LimelightConstants.kLimelimelight);
    if(m_visionSub.getHorizontalAngle() > 0) {
      turn_dir = -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_visionSub.estimateDistance() > 3.8) { // Drives towards the target
      double angle_offset = m_visionSub.getHorizontalAngle();
      kPower = MathUtil.clamp(m_angleController.calculate(-angle_offset, turn_dir * kOffsetAngle), -1.0, 1.0);
      m_drivetrainSub.arcadeDrive(-0.6, kPower * kMaxPower);
    } else {
      double angle_offset = m_visionSub.getHorizontalAngle(); // Directly aligns with target when close
      kPower = MathUtil.clamp(m_angleController.calculate(-angle_offset, 0.0), -1.0, 1.0);
      m_drivetrainSub.arcadeDrive(-0.4, kPower * kMaxPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drivetrainSub.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_visionSub.hasTarget()) {
      return true;
    }
    return false;
  }
}
