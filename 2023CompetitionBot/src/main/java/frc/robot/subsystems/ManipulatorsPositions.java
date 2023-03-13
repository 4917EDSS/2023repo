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
  HIGH_CUBE(157, 238000, 0), // Cube high drop-off position 
  MEDIUM_CUBE(78, 158000, 0), // Cube mid drop-off position
  LOW_CUBE(217, 126000, 0), // Cube low drop-off position
  GROUND_CUBE(271, 100000, 0), // Cube ground pick-up position
  SINGLE_STATION_CUBE(250, 0, 0), // Cube single (side) station pick-up position
  DOUBLE_STATION_CUBE(8.8333, -175041, 28.3569), // Cube double station pick-up position

  // Cone Positions
  HIGH_CONE(55.57191, 239449, 18.78565), // Cone high drop-off position 
  MEDIUM_CONE(0.14285, 175764, 3.666), // Cone mid drop-off position 
  LOW_CONE(105.16, 126224, 15.380), // Cone low drop-off position 
  GROUND_CONE(256, 100000, 0), // Cone ground pick-up position
  SINGLE_STATION_CONE(250, 0, 0), // Cone single (side) station pick-up position
  DOUBLE_STATION_CONE(0, -198943, 46.380); // Cone double station pick-up position


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
