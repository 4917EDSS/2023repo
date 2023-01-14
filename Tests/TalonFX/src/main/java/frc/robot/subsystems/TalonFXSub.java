// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFXSub extends SubsystemBase {
 TalonFX m_falcon500 = new TalonFX(15);
  /** Creates a new TalonFXSub. */
  public TalonFXSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setMotorPower(double power) {
    m_falcon500.set(ControlMode.PercentOutput, power);
  }

}
