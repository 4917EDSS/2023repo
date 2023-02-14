// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum IntakePositions {
  HIGH(-93, 40),
  MEDIUM(-62, 30),
  LOW(-36, 42),
  GROUND(-62, 60),
  STATION(60, 0),
  START(0, 0);

  public final double armEncoder;
  public final double mastEncoder;

  IntakePositions(double armEncoder, double mastEncoder) {
    this.armEncoder = armEncoder;
    this.mastEncoder = mastEncoder;
  }
}
