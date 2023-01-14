// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Documentation is located at https://docs.limelightvision.io/en/latest/getting_started.html - Read it, it's not that complex
/*
  **Hardware info** 

  The limelight has a 12V input but works at 6V, the LEDs brightness stays constant down to 7v

  Run the wires to the PDP slot (NOT the VRM)
  
  The limelight connects to the robot with an ethernet cable


  **Imaging**

  How to image the limelight in 5 easy steps

  1. Download the latest drivers and flashing stuff from limelight's download page: https://limelightvision.io/pages/downloads
  2. Install Balena Etcher and open it
  3. Connect the limelight through a usb (MAKE SURE POWER CABLE IS UNPLUGGED FROM THE LIMELIGHT)
  4. Select the .zip driver in Balena Etcher as the driver and select the limelight as the device
  5. Click flash and wait for it to finish; you can unplug the limelight once its done


  Note: Don't connect the Limelight to usb at any time besides flashing because it enters "flash mode" and work do anything else
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class LimelightSub extends SubsystemBase {
  /** Creates a new LimelightSub. */

  // Limelight uses NetworkTables for all of the data
  private final NetworkTable limelight;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  public LimelightSub() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    ta = limelight.getEntry("ta");
    tv = limelight.getEntry("tv");
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
