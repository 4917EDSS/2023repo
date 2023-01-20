// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSub extends SubsystemBase {
  /** Creates a new DrivetrainSub. */
  private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public DrivetrainSub() {
    m_shifter.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shift(boolean isHigh) {
    //Shifts the shifter solenoid according to the isHigh parameter, true for high, false for low.
    m_shifter.set(isHigh);
  }
}
