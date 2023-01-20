// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Hardware and API Info:
 * https://www.studica.ca/en/navx-2-mxp-robotics-navigation-sensor
 * 
 * As of early January 2023, the info on Kauai Labs' site was out of date, still showing last year's
 * sample code and libraries.
 * 
 */
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXSub extends SubsystemBase {
  // The port specified how the NavX is connected to the roboRIO.
  // In this case it's via the SPI port on the MXP connector on top of the robotRIO.
  AHRS m_navX = new AHRS(SPI.Port.kMXP);

  /** Creates a new NavXSub. */
  public NavXSub() {
    m_navX.reset(); // Set the yaw angle to 0
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAngle() {
    /*
    Other useful methods

    m_navX.getRoll(); // Roll -180 to 180 deg
    m_navX.getPitch(); // Pitch -180 to 180 deg
    m_navX.getYaw();  // Yaw -180 to 180 deg
    m_navX.getRate(); // Rate of rotation
    */
    return m_navX.getAngle(); // Cumumlative angle change
  }
}