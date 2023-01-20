// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Hardware Info:
 * https://docs.wpilib.org/en/latest/docs/software/can-devices/pneumatics-control-module.html
 * 
 * API Info:
 * https://docs.wpilib.org/en/latest/docs/software/hardware-apis/pneumatics/pneumatics.html#solenoid-control
 * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Solenoid.html
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SolenoidSub extends SubsystemBase {
  // The type refers to the version of the module that we are using (CTRE's PCM or Rev's Pneumatic Hub)
  // The channel number specifies which of the module/hub channel the valve is connected to.
  Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIds.solenoidId);

  /** Creates a new SolenoidSub. */
  public SolenoidSub() {
    // Put the valve/solenoid in a known default state
    m_solenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSolenoid(boolean isOn) {
    m_solenoid.set(isOn);
  }
}
