// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CanIds {
    public final static int kDriveMotorFL = 1; // Front left motor
    public final static int kTurningMotorFL = 2;
    public final static int kDriveMotorFR = 3; // Front right motor
    public final static int kTurningMotorFR = 4;
    public final static int kDriveMotorBL = 5; // Back left motor
    public final static int kTurningMotorBL = 6;
    public final static int kDriveMotorBR = 7; // Back right motor
    public final static int kTurningMotorBR = 8;
    // 9
    // 10
    public final static int kEncoderFL = 11; // All turning encoders
    // 12
    public final static int kEncoderBL = 13;
    // 14
    public final static int kEncoderFR = 15;
    // 16
    public final static int kEncoderBR = 17;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveModuleConstants {
    public static final double kAbsoluteEncoderOffsetFL = 4.229176;
    public static final double kAbsoluteEncoderOffsetFR = 1.50;
    public static final double kAbsoluteEncoderOffsetBL = 3.221353;
    public static final double kAbsoluteEncoderOffsetBR = 2.911489;
  }

  public static class DriveConstants {
    public static final double kTrackWidth = 0.678; // meters
    public static final double kWheelBase = 0.633; // meters
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  public static class AutoConstants {
    public static double kMaxSpeedMetersPerSecond = 0.5; // Increase to 3 for faster speed in auto
    public static double kMaxAccelerationMetersPerSecondSquared = 0.5; // increase to 3 for faster accelleration in auto
    public static double kMaxAngularSpeedRadiansPerSecond = 0.1;
    public static double kMaxAngularAccelerationRadiansPerSecondSquared = 0.1;
    public static double kPThetaController = 1.0;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static double kPXController = 1.0;
    public static double kPYController = 1.0;
  }
}
