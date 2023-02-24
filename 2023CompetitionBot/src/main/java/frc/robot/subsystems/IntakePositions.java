// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum IntakePositions {
  START(0, 0),
  // We are setting the values to zero, so that people use the cube or cone positions.
  HIGH(0, 0), 
  MEDIUM(0, 0), 
  LOW(0, 0), 
  GROUND(0, 0), 
  SINGLE_STATION(0, 0), 
  DOUBLE_STATION(0, 0),

  // Cone Positions
  HIGH_CONE(-93, 40), 
  MEDIUM_CONE(-62, 30), 
  LOW_CONE(-36, 42), 
  GROUND_CONE(-62, 60), 
  SINGLE_STATION_CONE(60, 0), 
  DOUBLE_STATION_CONE(3, 25),

  // Cube Positions
  HIGH_CUBE(-92, 40), 
  MEDIUM_CUBE(-61, 30), 
  LOW_CUBE(-35, 42), 
  GROUND_CUBE(-61, 60), 
  SINGLE_STATION_CUBE(59, 0), 
  DOUBLE_STATION_CUBE(61, 64);


  public final double armEncoder;
  public final double mastEncoder;

  IntakePositions(double armEncoder, double mastEncoder) {
    this.armEncoder = armEncoder;
    this.mastEncoder = mastEncoder;
  }

  public static IntakePositions convert(IntakePositions unConverted, boolean coneMode) {
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

    }
    return unConverted;
  }

}
