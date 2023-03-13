// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum ManipulatorsPositions {
  // Mast, Arm, Intake wrist encoder values for all positions
  START(0, 0, 0), // Position where robot started or hit the reset-encoder limit switches
  HOME(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Carrying position (inside frame)

  // These enum entries should only be used to convert to a cube/cone value
  HIGH(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values
  MEDIUM(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values
  LOW(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values
  GROUND(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values
  SINGLE_STATION(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values
  DOUBLE_STATION(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), // Gets replaced by cube/cone values

  // Cube Positions
  HIGH_CUBE(24.285, 218812, 0.119), // Cube high drop-off position 
  MEDIUM_CUBE(0.857, 136899, 4.19047), // Cube mid drop-off position
  LOW_CUBE(0.857, 56313, 9.214), // Cube low drop-off position
  GROUND_CUBE(42.475, 74607, 13.07), // Cube ground pick-up position
  SINGLE_STATION_CUBE(250, 0, 0), // Cube single (side) station pick-up position
  DOUBLE_STATION_CUBE(0, -167851, 9.833), // Cube double station pick-up position

  // Cone Positions
  HIGH_CONE(13.19, 239423, 9.738), // Cone high drop-off position 
  MEDIUM_CONE(0.333, 185120, 9.595), // Cone mid drop-off position 
  LOW_CONE(1.07, 101048, 12.857), // Cone low drop-off position 
  GROUND_CONE(23.618, 109151, 16.1904), // Cone ground pick-up position
  SINGLE_STATION_CONE(250, 0, 0), // Cone single (side) station pick-up position
  DOUBLE_STATION_CONE(0, -203081, 26.99); // Cone double station pick-up position


  // This is where the encoder values are stored for each enum entry
  public final double mastEncoder;
  public final double armEncoder;
  public final double wristEncoder;

  ManipulatorsPositions(double mastEncoder, double armEncoder, double wristEncoder) {
    this.mastEncoder = mastEncoder;
    this.armEncoder = armEncoder;
    this.wristEncoder = wristEncoder;
  }

  /** Modifies position based on which mode is enabled (Cone or Cube) */
  public static ManipulatorsPositions convert(ManipulatorsPositions unConverted, boolean coneMode) {
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
