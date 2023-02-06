// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Read the doc for more info: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.simple.*;
import org.json.simple.parser.*;
import frc.robot.Constants;

public class VisionSub extends SubsystemBase {

  // Variables
  private final NetworkTable m_limelight;
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;
  private NetworkTableEntry m_getpipe;
  private NetworkTableEntry m_pipeline; // Use constants for pipeline
  private NetworkTableEntry m_json;

  private int m_pipe = 0;

  private final double kSlope = 0.7667; // Conversion from robot space to meters

  /** Creates a new VisionSub. */
  public VisionSub() {
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_limelight.getEntry("tx");
    m_ty = m_limelight.getEntry("ty");
    m_ta = m_limelight.getEntry("ta");
    m_tv = m_limelight.getEntry("tv");
    m_tid = m_limelight.getEntry("tid");
    m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");
    m_json = m_limelight.getEntry("json");
  }

  public double getHorizontalAngle() { // Horizontal offset between -27 to 27 degrees or -29.8 to 29.8 degrees
    return m_tx.getDouble(0.0);
  }

  public double getVerticalAngle() { // Horizontal offset between -20.5 to 20.5 degrees or -24.85 to 24.85 degrees
    return m_ty.getDouble(0.0);
  }

  public double getTargetArea() { // Get area of target in %
    return m_ta.getDouble(0.0);
  }

  public boolean hasTarget() { // Returns true if any valid targets exist
    return (m_tv.getDouble(0.0) == 0.0) ? false : true;
  }

  public int getVisionMode() { // Gets current vision pipeline number
    Long val = m_tid.getInteger(0);
    return val.intValue();
  }

  public int getPrimaryID() { // Get primary apriltag ID (-1 means nothing)
    Long val = m_tid.getInteger(-1);
    return val.intValue();
  }

  public void setPipeline(int line) { // Set the currect pipeline (NO_VISION, LIMELIGHT, or APRILTAG)
    m_pipeline.setNumber(line);
  }

  public double getDistance() { // Returns distance in meter, 0 if no distance [ Must have apriltag pipeline enabled]

    if(getVisionMode() == Constants.LimelightConstants.kApriltag) {
      JSONParser parser = new JSONParser();
      String temp_json = m_json.getString("");
      JSONObject json_data;
      JSONArray result_data;
      double distance = 0.0;

      try {
        json_data = (JSONObject) parser.parse(temp_json); // Go through the json dump to get the tag Z position
        json_data = (JSONObject) json_data.get("Results");

        result_data = (JSONArray) json_data.get("Fiducial");
        json_data = (JSONObject) result_data.get(0);
        result_data = (JSONArray) json_data.get("t6t_cs");
        distance = (double) result_data.get(2);

      } catch (Exception e) {
        json_data = null;
        distance = -0.0; // -0.0 means error
        //System.out.println(e);
      }
      return distance * kSlope; // Convert to meters
    }
    return 0.0; // No Apriltag vision
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Horizontal Angle", getHorizontalAngle());
    SmartDashboard.putNumber("Vertical Angle", getVerticalAngle());
    SmartDashboard.putNumber("Target Area", getTargetArea());
    SmartDashboard.putBoolean("Target Available", hasTarget());
    SmartDashboard.putNumber("Apriltag ID", getPrimaryID());
    m_pipe = (int) SmartDashboard.getNumber("Pipeline", m_pipe);
    SmartDashboard.putNumber("Pipeline", m_pipe);

    SmartDashboard.putNumber("Distance (m)", getDistance());
  }
}
