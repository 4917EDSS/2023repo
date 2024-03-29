// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Level;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
  public static final Level kLogLevel = Level.WARNING;
  public static boolean kEnhanceDashBoard = false;

  // HARDWARE IDs/PORTs MAPPING ///////////////////////////////////////////////
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public final static class CanIds {
    public final static int kMastMotor = 1;
    public final static int kLeftDriveMotor1 = 2;
    public final static int kLeftDriveMotor2 = 3;
    public final static int kLeftDriveMotor3 = 4;
    public final static int kRightDriveMotor1 = 5;
    public final static int kRightDriveMotor2 = 6;
    public final static int kRightDriveMotor3 = 7;
    public final static int kRotateMotor = 9;
    public final static int kIntakeMotor = 10;
    public final static int kArmMotorR = 12;
    public final static int kArmMotorL = 13;
  }

  public final static class SolenoidIds {
    public final static int kShifter = 1;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }

  public final static class DioIds {
    public final static int kMastLimitPort = 0;
    public final static int kArmLimitPort = 1;
    public final static int kIntakeLimitPort = 2;
  }

  public final static class AnalogInputIds {
    public final static int kGamePieceSensorPort = 0;
    public final static int kGamePieceSensorPortRight = 1;
  }


  // OTHER GLOBAL CONSTANTS ///////////////////////////////////////////////////
  public final static class LimelightConstants {
    public final static int kNoVision = 0;
    public final static int kLimelimelight = 1;
    public final static int kApriltag = 2;
  }

  public final static class VisionConstants {
    public final static double kMountAngle = 0.0; // All measurements in meters
    public final static double kHighTapeHeight = 1.06;
    public final static double kLowTapeHeight = 0.56;
    public final static double kLensHeight = 0.0;
  }
}
