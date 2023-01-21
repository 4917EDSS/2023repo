// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*

  Documentation at: https://docs.wpilib.org/en/latest/docs/software/hardware-apis/pneumatics/pneumatics.html#double-solenoids-in-wpilib

  Hardware Info

 */


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// using this import allows for the "kOff", "kForward" and "kReverse" values

// Creates a new Double Solenoid object
public class DoubleSolenoidsSub extends SubsystemBase {
  /** Creates a new DoubleSolenoidsSub. */
  DoubleSolenoid m_doubleSolenoidPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  public DoubleSolenoidsSub() {
    m_doubleSolenoidPCM.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setValve(Value value) {
    m_doubleSolenoidPCM.set(value);
  }
}
