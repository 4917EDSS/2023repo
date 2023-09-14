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
    public final static int kDriveMotorRL = 5; // Back left motor
    public final static int kTurningMotorRL = 6;
    public final static int kDriveMotorRR = 7; // Back right motor
    public final static int kTurningMotorRR = 8;
    // 9
    // 10
    public final static int kEncoderFL = 11; // All turning encoders
    // 12
    public final static int kEncoderRL = 13;
    // 14
    public final static int kEncoderFR = 15;
    // 16
    public final static int kEncoderRR = 17;
  }


  public static final class DriveConstants {
    public static final boolean kDriveEncoderReversedFL = true;
    public static final boolean kDriveEncoderReversedFR = true;
    public static final boolean kDriveEncoderReversedRL = true;
    public static final boolean kDriveEncoderReversedRR = true;

    public static final boolean kTurnEncoderReversedFL = false;
    public static final boolean kTurnEncoderReversedFR = false;
    public static final boolean kTurnEncoderReversedRL = false;
    public static final boolean kTurnEncoderReversedRR = false;

    public static final double kTrackWidth = 0.678;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.633;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kAbsoluteEncoderOffsetFL = 4.229176;
    public static final double kAbsoluteEncoderOffsetFR = 0.15;
    public static final double kAbsoluteEncoderOffsetRL = 3.221353;
    public static final double kAbsoluteEncoderOffsetRR = 2.911489;

    public static final boolean kGyroReversed = false;

    // To-Do: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // public static final double ksVolts = 1;
    // public static final double kvVoltSecondsPerMeter = 0.8;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    // TODO: Should this be module constants or drive constants
    public static final double kMaxSpeedMetersPerSecond = 18 * 12 / 39.3701; /* 18ft/s in m/s for TalonFX */

    /** Minimum requested speed value (m/s) that manual control will react to */
    public static final double kSpeedDeadband = 0.1;
  }


  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI; // TODO: Check these
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI; // TODO: Check these

    public static final int kTurnEncoderCPR = 4096;

    /** Millimeters per tick (measured) */
    public static final double kDriveEncoderDistancePerPulse = (1000.0 / 51782);

    /** Conversion factor for mm/100ms to m/s */
    public static final double kDriveVelocityFactor = (10.0 / 1000.0);

    /** Radions per pulse (calculated) */
    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) kTurnEncoderCPR;

    public static final double kPModuleDriveController = 0.5;
    public static final double kPModuleTurningController = 0.5;
  }


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
