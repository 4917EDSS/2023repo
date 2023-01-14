// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Java API: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxExamples extends SubsystemBase {
  private static final int leftDeviceID = 2; 

  CANSparkMax m_motor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
  

  /** Creates a new sparkMaxExamples. */
  public SparkMaxExamples() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    m_motor.set(power);
  }
}
