// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Read the doc for more info: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSub extends SubsystemBase {
  /** Creates a new VisionSub. */

  // Variables
  private final NetworkTable limelight;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry tid;
  private NetworkTableEntry getpipe;
  private NetworkTableEntry pipeline; // Use constants for pipeline

  private int pipe = 0;

  public VisionSub() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    ta = limelight.getEntry("ta");
    tv = limelight.getEntry("tv");
    tid = limelight.getEntry("tid");
    getpipe = limelight.getEntry("getpipe");
    pipeline = limelight.getEntry("pipeline");
  }

  public double getHorizontalAngle() { // Horizontal offset between -27 to 27 degrees or -29.8 to 29.8 degrees
    return tx.getDouble(0.0);
  }

  public double getVerticalAngle() { // Horizontal offset between -20.5 to 20.5 degrees or -24.85 to 24.85 degrees
    return ty.getDouble(0.0);
  }

  public double getTargetArea() { // Get area of target in %
    return ta.getDouble(0.0);
  }

  public boolean hasTarget() { // Returns true if any valid targets exist
    return (tv.getDouble(0.0) == 0.0) ? false : true;
  }

  public int getVisionMode() { // Gets current vision pipeline number
    return (int)getpipe.getNumber(0);
  }

  public int getPrimaryID() { // Get primary apriltag ID (-1 means nothing)
    return (int)tid.getNumber(-1);
  }

  public void setPipeline(int line) { // Set the currect pipeline (NO_VISION, LIMELIGHT, or APRILTAG)
    pipeline.setNumber(line);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Horizontal Angle",getHorizontalAngle());
    SmartDashboard.putNumber("Vertical Angle",getVerticalAngle());
    SmartDashboard.putNumber("Target Area",getTargetArea());
    SmartDashboard.putBoolean("Target Available",hasTarget());
    SmartDashboard.putNumber("Apriltag ID",getPrimaryID());
    pipe = (int)SmartDashboard.getNumber("Pipeline", pipe);
    SmartDashboard.putNumber("Pipeline", pipe);
  }
}
