// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Hardware info:
/*
Sources: https://docs.wpilib.org/en/latest/docs/software/hardware-apis/pneumatics/pneumatics.html#solenoid-control,
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Solenoid.html
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

public class Solenoids extends SubsystemBase {
  /** Creates a new Solenoids. */
  public Solenoids() {



    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);

    exampleSolenoidPCM.set(true);
    exampleSolenoidPCM.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
