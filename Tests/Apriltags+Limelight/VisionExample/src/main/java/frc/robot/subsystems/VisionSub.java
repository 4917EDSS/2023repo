// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Documentation is located at https://docs.limelightvision.io/en/latest/getting_started.html - Read it, it's not that complex
// Apriltag docs are at https://docs.limelightvision.io/en/latest/apriltags_in_2d.html
// The entire API and values for the limelight are at https://docs.limelightvision.io/en/latest/networktables_api.html
/*
  **Hardware info** 

  The limelight has a 12V input but works at 6V, the LEDs brightness stays constant down to 7v

  Run the wires to the PDP slot (NOT the VRM)
  
  The limelight connects to the robot with an ethernet cable

  After updating the firmware, you can access the web interface through the ethernet cable and 12v power supply

  The Web interface is located at http://10.te.am.11:5801 where te.am is the team number (ex 49.17)

  Apriltags and Limelight vision use the same functions for targeting so it's just a matter of changing the pipeline

  ** Setup **

  Following the Limelight Getting started doc:
  1. Upgrade the camera to the newest firmware if needed (Compmute Module)
  2. DON'T Setup a static IP address with bonjour (Static ip should only be for competitions as you won't be able to connect the limelight to your laptop) 
  Note: Limelight Finder sucks and never works, just go to limelight.local:5801
  3. Create a no vision, limelight, and apriltag pipeline
  4. Download the pipelines to save them on your computer

  For this subsystem the order should be
  pipeline 0 - no vision
  pipeline 1 - limelight
  pipeline 2 - apriltag

  There isn't currently an official way to get the distance to the apriltag, someone would have to calculate it themselves

  Just use getTargetArea to see how close it is. See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html for more info

 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class VisionSub extends SubsystemBase {
  /** Creates a new LimelightSub. */


  // Constants
  private final int NO_VISION = 0;
  private final int LIMELIGHT = 1;
  private final int APRILTAG = 2;
  // Limelight uses NetworkTables for all of the data
  private final NetworkTable limelight;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry tid;
  private NetworkTableEntry getpipe;
  private NetworkTableEntry pipeline;

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

  
  public double getHorizontalOffset() { // Horizontal offset between -27 to 27 degrees or -29.8 to 29.8 degrees
    return tx.getDouble(0.0);
  }

  public double getVerticalOffset() { // Horizontal offset between -20.5 to 20.5 degrees or -24.85 to 24.85 degrees
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

    
    SmartDashboard.putNumber("Horizontal Angle",getHorizontalOffset());
    SmartDashboard.putNumber("Vertical Angle",getVerticalOffset());
    SmartDashboard.putNumber("Target Area",getTargetArea());
    SmartDashboard.putBoolean("Target Available",hasTarget());
    SmartDashboard.putNumber("Apriltag ID",getPrimaryID());
    pipe = (int)SmartDashboard.getNumber("Pipeline", pipe);
    SmartDashboard.putNumber("Pipeline", pipe);
    // This method will be called once per scheduler run
  }
}
