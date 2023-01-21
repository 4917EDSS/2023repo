// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSub extends SubsystemBase {
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(Constants.DrivetrainCanIds.kLeftDriveMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(Constants.DrivetrainCanIds.kRightDriveMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  /** Creates a new DrivetrainSub. */
  private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public DrivetrainSub() {
    m_shifter.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void tankDrive(double leftPower, double rightPower){
    m_leftMotor1.set(leftPower);
    m_leftMotor2.set(leftPower);
    m_leftMotor3.set(leftPower);
    m_leftMotor1.set(-rightPower);
    m_leftMotor2.set(-rightPower);
    m_leftMotor3.set(-rightPower);
  }

  public void shift(boolean isHigh) {
    //Shifts the shifter solenoid according to the isHigh parameter, true for high, false for low.
    m_shifter.set(isHigh);
  }
}
