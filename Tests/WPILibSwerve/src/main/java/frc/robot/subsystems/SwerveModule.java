package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  /**
   * Constants for this module
   */
  public static final class ModuleConstants {
    public static final double kMaxDriveMPerS = 1.0; // TODO: To be measured
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 1 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 1 * Math.PI;

    //public static final int kDriveEncoderCPR = 2048; // For CTRE TalonFX built-in encoder
    //public static final double kWheelDiameterMeters = 0.1016; // 4"
    public static final double kDriveEncoderDistancePerPulseMmPerTick = (1000.0 / 51782); // Millimeters per tick
    public static final double kDriveVelocityFactor = (10.0 / 1000.0); // mm/100ms to m/s 

    public static final int kTurningEncoderCPR = 4096; // For CTRE CANCoder magnetic encoder
    // Assumes the encoders are on a 1:1 reduction with the module shaft.
    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) kTurningEncoderCPR; // Radians per pulse

    public static final double kPModuleTurningController = 100;
    public static final double kPModuleDriveController = 1;
  }

  // Member variables
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(1.0, 0, 0);
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  private double m_turningEncoderOffset;

  /**
   * Construct the SwerveModule
   * 
   * @param driveMotorId CAN ID for the drive motor
   * @param turningMotorId CAN ID for the turning motor
   * @param turningEncoderId CAN ID for the turning encoder
   * @param reverseTurningEncoderDirection Set to true to reverse the default sensor direction
   * @param turningEncoderOffsetRad Value encoder shows when wheel is facing forward
   */
  public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId,
      boolean reverseTurningEncoderDirection, double turningEncoderOffsetRad) {

    // Create all the hardware objects
    m_driveMotor = new TalonFX(driveMotorId);
    m_turningMotor = new CANSparkMax(turningMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningEncoder = new CANCoder(turningEncoderId);

    // Configure all the hardware objects as needed (including drive motor encoder)
    m_driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderDistancePerPulseMmPerTick);
    m_driveMotor.setInverted(true);

    m_turningEncoder.configSensorDirection(reverseTurningEncoderDirection); // False means positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder.
    m_turningEncoder.configFeedbackCoefficient(ModuleConstants.kTurningEncoderDistancePerPulse, "radians",
        SensorTimeBase.PerSecond);

    // Configure the PID controllers
    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turningEncoderOffset = turningEncoderOffsetRad;

    resetEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMPerS(), new Rotation2d(getTurningPositionAbsoluteRad()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getSelectedSensorPosition() / 1000.0,
        new Rotation2d(getTurningPositionAbsoluteRad()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if(Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningPositionAbsoluteRad()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        //m_drivePIDController.calculate(getDriveVelocityMPerS(), state.speedMetersPerSecond);
        state.speedMetersPerSecond / ModuleConstants.kMaxDriveMPerS;

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurningPositionAbsoluteRad(), state.angle.getRadians());


    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(turnOutput);
  }

  public void stop() {
    m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_turningMotor.set(0);
  }


  public void setPID(boolean isDrive, double p, double i, double d) {
    if(isDrive) {
      if(m_drivePIDController.getP() != p) {
        m_drivePIDController.setP(p);
      }
      if(m_drivePIDController.getI() != i) {
        m_drivePIDController.setI(i);
      }
      if(m_drivePIDController.getD() != d) {
        m_drivePIDController.setD(d);
      }
    } else {
      if(m_turningPIDController.getP() != p) {
        m_turningPIDController.setP(p);
      }
      if(m_turningPIDController.getI() != i) {
        m_turningPIDController.setI(i);
      }
      if(m_turningPIDController.getD() != d) {
        m_turningPIDController.setD(d);
      }
    }
  }

  public double getDrivePositionM() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getTurningPosition() {
    return m_turningMotor.getEncoder().getPosition();
  }

  public double getDriveVelocityMPerS() {
    return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveVelocityFactor;
  }

  public double getTurningVelocity() {
    return m_turningMotor.getEncoder().getVelocity();
  }

  public double getTurningPositionAbsoluteRad() {
    return m_turningEncoder.getPosition() - m_turningEncoderOffset;
  }

  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0.0);
    m_turningMotor.getEncoder().setPosition(getTurningPositionAbsoluteRad());
  }
}
