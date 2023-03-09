// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum ManipulatorsPositions {
  START(0, 0, 0), HOME(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough),
  // We are setting the values to zero, so that people use the cube or cone positions.
  HIGH(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), MEDIUM(MastSub.kVertical, ArmSub.kVertical,
      IntakeSub.kWristThrough), LOW(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), GROUND(
          MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough), SINGLE_STATION(MastSub.kVertical,
              ArmSub.kVertical,
              IntakeSub.kWristThrough), DOUBLE_STATION(MastSub.kVertical, ArmSub.kVertical, IntakeSub.kWristThrough),

  // Cone Positions
  HIGH_CONE(197, 248000, 0), MEDIUM_CONE(78, 203000, 0), // Needs to br found
  LOW_CONE(217, 126000, 0), // Needs to br found
  GROUND_CONE(256, 100000, 0), SINGLE_STATION_CONE(250, 0, 0), // Needs to br found
  DOUBLE_STATION_CONE(61.834, -180802, 41.166),

  // Cube Positions
  HIGH_CUBE(157, 238000, 0), MEDIUM_CUBE(78, 158000, 0), // Needs to br found
  LOW_CUBE(217, 126000, 0), // Needs to br found
  GROUND_CUBE(271, 100000, 0), SINGLE_STATION_CUBE(250, 0, 0), // Needs to br found
  DOUBLE_STATION_CUBE(1.9286, -18080, 22.143);


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
