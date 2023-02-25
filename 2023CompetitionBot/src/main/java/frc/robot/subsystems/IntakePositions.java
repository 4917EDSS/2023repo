// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum IntakePositions {
  START(0, 0),
// We are setting the values to zero, so that people use the cube or cone positions.
  HIGH(-171291, 0),
  MEDIUM(-136108, 0),
  LOW(-78171, 0),
  GROUND(-122139, 0),
  STATION(123198, 0),
  //87205


// Cone Positions
  HIGH_CONE(-171291, 40),
  MEDIUM_CONE(-136108, 30),
  LOW_CONE(-78171, 42),
  GROUND_CONE(-122139, 60),
  STATION_CONE(123198, 0),

// Cube Positions
  HIGH_CUBE(-171291, 40),
  MEDIUM_CUBE(-136108, 30),
  LOW_CUBE(-78171, 42),
  GROUND_CUBE(-122139, 60),
  STATION_CUBE(123198, 0);

  public final double armEncoder;
  public final double mastEncoder;

  IntakePositions(double armEncoder, double mastEncoder) {
    this.armEncoder = armEncoder;
    this.mastEncoder = mastEncoder;
  }

  public static IntakePositions convert(IntakePositions unConverted, boolean coneMode){
  // Converts positions depending on which mode is enabled (Cone, Cube)
    switch(unConverted){
      case HIGH:
        if(coneMode){
          return HIGH_CONE;
        } else {
          return HIGH_CUBE;
        }

      case MEDIUM:
        if(coneMode){
          return MEDIUM_CONE;
        } else {
          return MEDIUM_CUBE;
        }

      case LOW:
        if(coneMode){
          return LOW_CONE;
        } else {
          return LOW_CUBE;
        }
      
      case GROUND:
        if(coneMode){
          return GROUND_CONE;
        } else {
          return GROUND_CUBE;
        }

      case STATION:
       if(coneMode){
          return STATION_CONE;
        } else {
          return STATION_CUBE;
        }
        
    }
    return unConverted;
  }

}
