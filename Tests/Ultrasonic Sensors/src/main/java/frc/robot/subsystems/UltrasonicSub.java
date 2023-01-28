// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*

Documentation at https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/ultrasonics-software.html

Hardware info:

 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Ultrasonic;

public class UltrasonicSub extends SubsystemBase {
  /** Creates a new UltrasonicSub. */
  public UltrasonicSub() {
    // Creates a ping-response Ultrasonic object
    Ultrasonic m_ultrasonicSensor = new Ultrasonic(1, 2);

    // Reads the distance in millimeters
    double distanceMillimeters = m_ultrasonicSensor.getRangeMM(); 
    
    System.out.println(distanceMillimeters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
