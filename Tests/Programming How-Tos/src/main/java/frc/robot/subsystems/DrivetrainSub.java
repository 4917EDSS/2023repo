// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  private final CANSparkMax m_leftMotor = 
      new CANSparkMax(Constants.CanIds.kLeftDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = 
      new CANSparkMax(Constants.CanIds.kRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor.set(leftPower);
    m_rightMotor.set(-rightPower);
  }
}
