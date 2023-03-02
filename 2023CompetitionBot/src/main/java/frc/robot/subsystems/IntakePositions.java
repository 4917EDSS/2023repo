// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum IntakePositions {
  START(0, 0, 0),
  // We are setting the values to zero, so that people use the cube or cone positions.
  HIGH(0, 0, 0), 
  MEDIUM(0, 0, 0), 
  LOW(0, 0, 0), 
  GROUND(0, 0, 0), 
  SINGLE_STATION(0, 0, 0), 
  DOUBLE_STATION(0, 0, 0),

  // Cone Positions
  HIGH_CONE(-171291, 40, 0), 
  MEDIUM_CONE(-136108, 30, 0), 
  LOW_CONE(-78171, 42, 0), 
  GROUND_CONE(-122139, 60, 0), 
  SINGLE_STATION_CONE(87205, 0, 0), 
  DOUBLE_STATION_CONE(123198, 25, 0),

  // Cube Positions
  HIGH_CUBE(-171291, 40, 0), 
  MEDIUM_CUBE(-136108, 30, 0), 
  LOW_CUBE(-78171, 42, 0), 
  GROUND_CUBE(-122139, 60, 0), 
  SINGLE_STATION_CUBE(87205, 0, 0), 
  DOUBLE_STATION_CUBE(123198, 64, 0);


  public final double armEncoder;
  public final double mastEncoder;
  public final double wristEncoder;

  IntakePositions(double armEncoder, double mastEncoder, double wristEncoder) {
    this.armEncoder = armEncoder;
    this.mastEncoder = mastEncoder;
    this.wristEncoder = wristEncoder;
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
      default:
        return unConverted;

    }
  }

}