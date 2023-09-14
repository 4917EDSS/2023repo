// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANCoder m_turnEncoder;
  private final double m_turningEncoderOffset;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Construct the SwerveModule
   * 
   * @param driveMotorId CAN ID for the drive motor
   * @param turnMotorId CAN ID for the turning motor
   * @param turnEncoderId CAN ID for the turning encoder
   * @param driveEncoderReversed Set to true to reverse the default sensor direction
   * @param turnEncoderReversed Set to true to reverse the default sensor direction
   * @param turnEncoderOffsetRad Value encoder shows when wheel is facing forward
   */
  public SwerveModule(
      int driveMotorId,
      int turnMotorId,
      int turnEncoderId,
      boolean driveEncoderReversed,
      boolean turnEncoderReversed,
      double turnEncoderOffsetRad) {

    m_driveMotor = new TalonFX(driveMotorId);
    m_turnMotor = new CANSparkMax(turnMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Drive encoder is built into the TalonFX do no need to create it
    m_turnEncoder = new CANCoder(turnEncoderId);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Configure drive motor
    m_driveMotor.setInverted(driveEncoderReversed);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    // Configure turn motor
    m_turnMotor.setIdleMode(IdleMode.kBrake);

    // Set up the encorder
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configFeedbackCoefficient(ModuleConstants.kTurningEncoderDistancePerPulse, "radians",
        SensorTimeBase.PerSecond);
    m_turnEncoder.configSensorDirection(turnEncoderReversed); // False means positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder.
    m_turningEncoderOffset = turnEncoderOffsetRad;


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if(Math.abs(desiredState.speedMetersPerSecond) < DriveConstants.kSpeedDeadband) {
      setMotors(0.0, 0.0);
      return;
    }


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnPosition()));

    // Calculate the drive output from the drive PID controller.
    //final double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
    final double driveOutput = state.speedMetersPerSecond / 1.0;//DriveConstants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurnPosition(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    setMotors(driveOutput, turnOutput);
  }

  /**
   * Sets the drive and turn motor power
   * 
   * @param drivePower Drive power (-1.0 to 1.0)
   * @param turnPower Turn power (-1.0 to 1.0)
   */
  public void setMotors(double drivePower, double turnPower) {
    m_driveMotor.set(ControlMode.PercentOutput, drivePower);
    m_turnMotor.set(turnPower);
  }

  /**
   * Returns the drive distance in m
   * 
   * @return The drive distance in meters
   */
  public double getDrivePosition() {
    return m_driveMotor.getSelectedSensorPosition() / 1000.0; /* Sensor position reported in mm */
  }

  /**
   * Returns the drive velocity in m/s
   * 
   * @return The drive velocity in meters per second
   */
  public double getDriveVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveVelocityFactor;
  }

  /**
   * Returns the turn position relative to the front of the robot
   * 
   * @return The angle of the wheel in radians
   */
  public double getTurnPosition() {
    double position = m_turnEncoder.getAbsolutePosition() - m_turningEncoderOffset;

    // Compensate for the offset's effect on the absolute encorder roll-over
    // Want values from -PI to +PI (radians)
    if(position < -Math.PI) {
      position += (2 * Math.PI);
    } else if(position > Math.PI) {
      position -= (2 * Math.PI);
    }

    return position;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0.0);
    // m_turningMotor.getEncoder().setPosition(0.0); // Can't reset this absolute encoder
  }
}
