// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public final static class CanIds {
    // public final static int kLeftDriveMotor = 1;	// This gives the code the network address of the motors
    public final static int kArmMotor = 8;
    public final static int kMastMotor = 5;
    /*public final static int kTopDriveMotor = 1;	// This gives the code the network address of the motors
    public final static int kShaftDriveMotor = 2;*/
  }

  public final static class LimelightConstants {
    public final int kNoVision = 0;
    public final int kLimelimelight = 1;
    public final int kApriltag = 2;
  }

  public final static class DrivetrainCanIds {
    public final static int kLeftDriveMotor1 = 1;
    public final static int kLeftDriveMotor2 = 2;
    public final static int kLeftDriveMotor3 = 3;   
    public final static int kRightDriveMotor1 = 4;
    public final static int kRightDriveMotor2 = 6;
    public final static int kRightDriveMotor3 = 7;
  }

  public final static class SolenoidIds {
    public final static int kShifter = 1;
    public final static int kGripperCylinder = 2;
  }


}