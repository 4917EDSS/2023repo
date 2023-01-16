// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class DrivetrainSub extends SubsystemBase {
  private static final boolean kGyroReversed = false;
  private static final double kMaxSpeedMetersPerSecond = 3;
  private static final double kTrackWidth = 0.678;    // meters
  // Distance between centers of right and left wheels on robot
  private static final double kWheelBase = 0.633;     // meters
  // Distance between front and back wheels on robot
  private static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // Create the 4 swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CanIds.kDriveMotorFL,
      CanIds.kTurningMotorFL,
      CanIds.kEncoderFL,
      false);
  private final SwerveModule m_frontRight = new SwerveModule(
      CanIds.kDriveMotorFR,
      CanIds.kTurningMotorFR,
      CanIds.kEncoderFR,
      false);
  private final SwerveModule m_backLeft = new SwerveModule(
      CanIds.kDriveMotorBL,
      CanIds.kTurningMotorBL,
      CanIds.kEncoderBL,
      false);
  private final SwerveModule m_backRight = new SwerveModule(
      CanIds.kDriveMotorBR,
      CanIds.kTurningMotorBR,
      CanIds.kEncoderBR,
      false);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  SwerveDriveOdometry m_odometry = 
  new SwerveDriveOdometry(
      kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
  });

    // This method will be called once per scheduler run
    // SwerveModulePosition smp = m_frontLeft.getPosition();
    // SwerveModuleState sms = m_frontLeft.getState();

    // SmartDashboard.putNumber("FL Dist", (double)(smp.distanceMeters));
    // SmartDashboard.putNumber("FL Angle Deg", smp.angle.getDegrees());
    // SmartDashboard.putNumber("FL Speed mps", sms.speedMetersPerSecond);
    // SmartDashboard.putNumber("FL Angle Rad", sms.angle.getRadians());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }
}
