// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSub extends SubsystemBase {
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor1,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor2,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor3,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor1,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor2,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor3,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2,
      m_rightMotor3);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SolenoidIds.kShifter);

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {

    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_leftMotor3.setInverted(false);
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);
    m_rightMotor3.setInverted(true);

    m_shifter.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftPower, double rightPower) {
    m_drive.tankDrive(leftPower, rightPower);
  }

  public void arcadeDrive(double leftPower, double rightPower) {
    m_drive.arcadeDrive(leftPower, rightPower);
  }

  public void shift(boolean isHigh) {
    // Shifts the shifter solenoid according to the isHigh parameter, true for high,
    // false for low.
    m_shifter.set(isHigh);
  }

  // TODO #6: Finish implementing auto-shift.  Then enable it in DriveWithJoystick
  public void autoShift(){
    // if(!m_isAutoShift){
    //   return;
    // } else {
      
    //   double averageWheelSpeed = (getLeftVelocity() + getRightVelocity()) / 2.;

    //   if(fabs(averageWheelSpeed) > kshiftHighSpeed){
    //     shift(true);
    //   } else if (fabs(averageWheelSpeed) < kshiftLowSpeed){
    //     shift(false);
    //   }
    // }
  }
}
