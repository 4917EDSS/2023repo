// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*

Documentation at: https://docs.wpilib.org/en/latest/docs/software/hardware-apis/pneumatics/pneumatics.html#solenoid-control,
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Solenoid.html


---Hardware info---

Applies or vents pressure from a single output port


 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Solenoids extends SubsystemBase {
  /* Creates a new Solenoids. */
  Solenoid m_solenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  // Creates a new Solenoid object, sets the value to true
  public Solenoids() {
    m_solenoidPCM.set(false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setValve(boolean isOpen) {
    m_solenoidPCM.set(isOpen);
  }
}
