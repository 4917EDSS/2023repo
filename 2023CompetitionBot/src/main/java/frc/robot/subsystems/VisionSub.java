// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Read the doc for more info: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

package frc.robot.subsystems;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSub extends SubsystemBase {

  // Variables
  private final NetworkTable m_limelight;
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;
  //private NetworkTableEntry m_getpipe;
  private NetworkTableEntry m_pipeline; // Use constants for pipeline
  private NetworkTableEntry m_json;

  //private int m_pipe = 0;

  private final double kSlope = 0.7667; // Conversion from robot space to meters

  /** Creates a new VisionSub. */
  public VisionSub() {
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_limelight.getEntry("tx");
    m_ty = m_limelight.getEntry("ty");
    m_ta = m_limelight.getEntry("ta");
    m_tv = m_limelight.getEntry("tv");
    m_tid = m_limelight.getEntry("tid");
    //m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");
    m_json = m_limelight.getEntry("json");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Horizontal Angle", getHorizontalAngle());
    SmartDashboard.putNumber("Vertical Angle", getVerticalAngle());
    SmartDashboard.putNumber("Target Area", getTargetArea());
    SmartDashboard.putBoolean("Target Available", hasTarget());
    SmartDashboard.putNumber("Apriltag ID", getPrimaryID());
    //m_pipe = (int) SmartDashboard.getNumber("Pipeline", m_pipe);
    //SmartDashboard.putNumber("Pipeline", m_pipe);

    SmartDashboard.putNumber("Distance (m)", getDistance());
    SmartDashboard.putNumber("Distance X", getDistanceX());
    SmartDashboard.putNumber("Y angle", getRY());
    if((getRY()-getHorizontalAngle()) < -10.0) {
      SmartDashboard.putString("Side of apriltag", "Left"); // Robot on Left side of april tag
    }
    else if((getRY()-getHorizontalAngle()) > 10) {
      SmartDashboard.putString("Side of apriltag", "Right"); // Robot on right side of april tag
    }
    else {
      SmartDashboard.putString("Side of apriltag", "Center");
    }
  }

  /**
   * Use this method to reset all of the hardware and states to safe starting values
   */
  public void init() {
    setPipeline(Constants.LimelightConstants.kApriltag);
  }

  /**
   * This method puts the subsystem in a safe state when all commands are interrupted
   */
  public void interrupt() {

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
    double val = m_tid.getDouble(0.0);
    return (int)val;
  }

  public int getPrimaryID() { // Get primary apriltag ID (-1 means nothing)
    Long val = m_tid.getInteger(-1);
    return val.intValue();
  }

  public void setPipeline(int line) { // Set the currect pipeline (NO_VISION, LIMELIGHT, or APRILTAG)
    m_pipeline.setNumber(line);
  }
  public double getRY() { // Get the Y angle relative to the april tag
    if(1==1/*getVisionMode() == Constants.LimelightConstants.kApriltag*/) {
      JSONParser parser = new JSONParser();
      String temp_json = m_json.getString("");
      JSONObject json_data;
      JSONArray result_data;
      double yAngle = 0.0;

      try {
        json_data = (JSONObject) parser.parse(temp_json); // Go through the json dump to get the tag Z position
        json_data = (JSONObject) json_data.get("Results");

        result_data = (JSONArray) json_data.get("Fiducial");
        json_data = (JSONObject) result_data.get(0);
        result_data = (JSONArray) json_data.get("t6t_cs");
        yAngle = (double) result_data.get(4); // X value of apriltag

      } catch (Exception e) {
        json_data = null;
        yAngle = -0.0; // -0.0 means error
        //System.out.println(e);
      }
      return yAngle; // Convert to meters
    }
    return 0.0; // No Apriltag vision
  }
  public double getDistance() { // Returns distance in meters, 0 if no distance [ Must have apriltag pipeline enabled]

    if(1==1/*getVisionMode() == Constants.LimelightConstants.kApriltag*/) {
      JSONParser parser = new JSONParser();
      String temp_json = m_json.getString("");
      JSONObject json_data;
      JSONArray result_data;
      double dx = 0.0;
      double dz = 0.0;
      double distance = 0.0;

      try {
        json_data = (JSONObject) parser.parse(temp_json); // Go through the json dump to get the tag Z position
        json_data = (JSONObject) json_data.get("Results");

        result_data = (JSONArray) json_data.get("Fiducial");
        json_data = (JSONObject) result_data.get(0);
        result_data = (JSONArray) json_data.get("t6t_cs");
        dx = (double) result_data.get(0); // X value of apriltag
        dz = (double) result_data.get(2); // Z value of apriltag
        distance = Math.sqrt(dx * dx + dz * dz) * kSlope;

      } catch (Exception e) {
        json_data = null;
        distance = -0.0; // -0.0 means error
        //System.out.println(e);
      }
      return distance; // Convert to meters
    }
    return 0.0; // No Apriltag vision
  }

  public double getDistanceX() { // Get horizontal distance from target
    if(getVisionMode() == Constants.LimelightConstants.kApriltag) {
      JSONParser parser = new JSONParser();
      String temp_json = m_json.getString("");
      JSONObject json_data;
      JSONArray result_data;
      double dx = 0.0;

      try {
        json_data = (JSONObject) parser.parse(temp_json); // Go through the json dump to get the tag Z position
        json_data = (JSONObject) json_data.get("Results");

        result_data = (JSONArray) json_data.get("Fiducial");
        json_data = (JSONObject) result_data.get(0);
        result_data = (JSONArray) json_data.get("t6t_cs");
        dx = (double) result_data.get(0); // X value of apriltag

      } catch (Exception e) {
        json_data = null;
        dx = -0.0; // -0.0 means error
        //System.out.println(e);
      }
      return dx * kSlope; // Convert to meters
    }
    return 0.0; // No Apriltag vision
  }

}
