// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.ManipulatorSub;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSub extends SubsystemBase {
  /** Creates a new TemplateSub. */
  Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIds.solenoidId);
  public ManipulatorSub() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSolenoid(boolean isOn) {
    m_solenoid.set(isOn);
  }
}
