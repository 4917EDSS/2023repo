// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {

  private final double kShiftUpSpeed = 1.8; // meters per second
  private final double kShiftDownSpeed = 1.4; // meters per second

  private final double kEncoderRotationsToMeterLowGear = 5.0 / 204.5;
  private final double kEncoderRotationsToMeterHighGear = 5.0 / 129.8;

  private final CANSparkMax m_leftMotor1 =
      new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 =
      new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 =
      new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 =
      new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 =
      new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 =
      new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SolenoidIds.kShifter);

  boolean m_isAutoShift = true;

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {

    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_leftMotor3.setInverted(false);
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);
    m_rightMotor3.setInverted(true);

    m_shifter.set(false);

    zeroDrivetrainEncoders();

    setIsAutoShift(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmarterDashboard();
  }

  /** Use this method to reset all of the hardware and states to safe starting values */
  public void init() {
    //TODO: Add resets here.  Call from constructor.
  }

  /** This method puts the subsystem in a safe state when all commands are interrupted */
  public void interrupt() {
    // Nothing to do on interrupt at this time
  }

  private void zeroDrivetrainEncoders() {
    m_leftMotor1.getEncoder().setPosition(0);
  }

  private double getLeftMotorEncoder() {
    return m_leftMotor1.getEncoder().getPosition();
  }

  private double getRightMotorEncoder() {
    return m_rightMotor1.getEncoder().getPosition();
  }

  private double getLeftVelocity() {
    return m_leftMotor1.getEncoder().getVelocity() * getEncoderRotationsToMeterFactor() / 60.; // In meters per second
  }

  private double getRightVelocity() {
    return m_rightMotor1.getEncoder().getVelocity() * getEncoderRotationsToMeterFactor() / 60.; // In meters per second
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  private double getLeftEncoderDistanceM() {
    return getLeftMotorEncoder() * getEncoderRotationsToMeterFactor();
  }

  private double getRightEncoderDistanceM() {
    return getRightMotorEncoder() * getEncoderRotationsToMeterFactor();
  }

  public double getEncoderDistanceM() {
    return (getLeftEncoderDistanceM() + getRightEncoderDistanceM() / 2);
  }

  private double getEncoderRotationsToMeterFactor() {
    if(m_shifter.get()) {
      return kEncoderRotationsToMeterHighGear;
    } else {
      return kEncoderRotationsToMeterLowGear;
    }
  }

  public void setIsAutoShift(boolean autoShiftActive) {
    m_isAutoShift = autoShiftActive;
    return;
  }

  public boolean getisAutoShift() {
    return m_isAutoShift;
  }

  public void updateSmarterDashboard() {
    SmartDashboard.putNumber("left motor1 encoder", getLeftMotorEncoder());
    SmartDashboard.putBoolean("Auto Shift", (getisAutoShift()));
    SmartDashboard.putBoolean("Coasting", isCoasting());
    // TODO Add high and low gear on Smart Dashboard
    // SmartDashboard.putBoolean("High Gear", ())
  }

  public void tankDrive(double leftPower, double rightPower) {
    m_drive.tankDrive(leftPower, rightPower);
  }

  public void arcadeDrive(double fwdPower, double turnPower) {
    m_drive.arcadeDrive(fwdPower, turnPower);
  }

  public void shift(boolean isHigh) {
    // Shifts the shifter solenoid according to the isHigh parameter, true for high,
    // false for low.
    m_shifter.set(isHigh);
  }

  public boolean isCoasting() {
    return m_rightMotor3.getIdleMode().equals(IdleMode.kCoast);
  }

  public void setBrakeCmd(boolean isBrakeOn) {
    IdleMode mode;
    if(isBrakeOn) {
      tankDrive(0, 0);
      arcadeDrive(0, 0);
      mode = IdleMode.kBrake;
    } else {
      mode = IdleMode.kCoast;
    }

    m_leftMotor1.setIdleMode(mode);
    m_leftMotor2.setIdleMode(mode);
    m_leftMotor3.setIdleMode(mode);
    m_rightMotor1.setIdleMode(mode);
    m_rightMotor2.setIdleMode(mode);
    m_rightMotor3.setIdleMode(mode);
  }

  public void autoShift() {
    if(!m_isAutoShift) {
      return;
    } else {
      double averageWheelSpeed = (getLeftVelocity() + getRightVelocity()) / 2.;

      if(Math.abs(averageWheelSpeed) > kShiftUpSpeed) {
        shift(true);
      } else if(Math.abs(averageWheelSpeed) < kShiftDownSpeed) {
        shift(false);
      }
    }
  }
}
