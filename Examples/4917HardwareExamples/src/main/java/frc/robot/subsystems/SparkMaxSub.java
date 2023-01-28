// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SparkMaxSub extends SubsystemBase {
  // The ID is the CAN ID that the motor was programmed with (using the Rev software)
  // The motor type is always brushless for a Neo but the SparkMax controllers also support brushed motors.
  private CANSparkMax m_motor = new CANSparkMax(Constants.CanIds.sparkMaxId, MotorType.kBrushless);
 // private CANSparkMax m_encoder = new CANSparkMax(Constants.CanIds.sparkMaxId, );

  /** Creates a new SparkMaxSub. */
  public SparkMaxSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double frontLeftPosition = m_motor.getEncoder().getPosition();
    
  }

  public void setPower(double power) {
    m_motor.set(power);
    
    
  }
}
