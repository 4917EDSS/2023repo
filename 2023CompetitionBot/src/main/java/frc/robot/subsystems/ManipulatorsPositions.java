// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum ManipulatorsPositions {
  START(0, 0, 0),
  HOME(118,0,0),
  // We are setting the values to zero, so that people use the cube or cone positions.
  HIGH(0, 0, 0), 
  MEDIUM(0, 0, 0), 
  LOW(0, 0, 0), 
  GROUND(0, 0, 0), 
  SINGLE_STATION(0, 0, 0), 
  DOUBLE_STATION(0, 0, 0),

  // Cone Positions
  HIGH_CONE(300, 40, 0), 
  MEDIUM_CONE(250, 30, 0), 
  LOW_CONE(150, 42, 0), 
  GROUND_CONE(200, 60, 0), 
  SINGLE_STATION_CONE(250, 0, 0), 
  DOUBLE_STATION_CONE(300, 25, 0),

  // Cube Positions
  HIGH_CUBE(300, 40, 0), 
  MEDIUM_CUBE(250, 30, 0), 
  LOW_CUBE(150, 42, 0), 
  GROUND_CUBE(200, 60, 0), 
  SINGLE_STATION_CUBE(250, 0, 0), 
  DOUBLE_STATION_CUBE(300, 64, 0);


  public final double armEncoder;
  public final double mastEncoder;
  public final double wristEncoder;

  ManipulatorsPositions(double mastEncoder, double armEncoder, double wristEncoder) {
    this.mastEncoder = mastEncoder;
    this.armEncoder = armEncoder;
    this.wristEncoder = wristEncoder;
  }

  public static ManipulatorsPositions convert(ManipulatorsPositions unConverted, boolean coneMode) {
    // Converts positions depending on which mode is enabled (Cone, Cube)
    switch(unConverted) {
      case HIGH:
        if(coneMode) {
          return HIGH_CONE;
        } else {
          return HIGH_CUBE;
        }

      case MEDIUM:
        if(coneMode) {
          return MEDIUM_CONE;
        } else {
          return MEDIUM_CUBE;
        }

      case LOW:
        if(coneMode) {
          return LOW_CONE;
        } else {
          return LOW_CUBE;
        }

      case GROUND:
        if(coneMode) {
          return GROUND_CONE;
        } else {
          return GROUND_CUBE;
        }

      case SINGLE_STATION:
        if(coneMode) {
          return SINGLE_STATION_CONE;
        } else {
          return SINGLE_STATION_CUBE;
        }

      case DOUBLE_STATION:
        if(coneMode) {
          return DOUBLE_STATION_CONE;
        } else {
          return DOUBLE_STATION_CUBE;
        }
      default:
        return unConverted;

    }
  }

}