// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    System.out.println("Im in Init");
    m_ledStrip.setLength(m_ledBuffer.getLength());
    setColor(1, 128, 0, 128);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

   // System.out.println("Im in periodiic");
    setColor(1, 128, 0, 128);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if(m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}


  @Override
  public void teleopInit() {
    if(m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void setColor(int i, int r, int g, int b) {
    // Sets the RGB color of an LED, or of all of them.
    // The r, g, and b parameters are quite self-explanatory (representing the r, g,
    // and b in rgb color.)
    // The i parameter represents the index of the LED whose color should be
    // changed, from 0 to kLedStripLength-1.
    // If i is negative, all LEDs on the strip will be changed.
    // Note that the rgb color system uses 3 values (r, g, and b) to represent
    // color.
    // r represents the amount of red (from 0 to 255) g represents the amount of
    // green (from 0 to 255) and b represents the amount of blue (from 0 to 255.)
    // Colors are made by combining various amounts of these 3 basic colors.
    if(r < 0) {
      r = 0;
    }
    if(r > 255) {
      r = 255;
    }
    if(g < 0) {
      g = 0;
    }
    if(g > 255) {
      g = 255;
    }
    if(b < 0) {
      b = 0;
    }
    if(b > 255) {
      b = 255;
    }
    //
    // m_rr = r;
    // m_gg = g;
    // m_bb = b;
    //
    if(i < 0) {
      for(int led = 0; led < m_ledBuffer.getLength(); led++) {
        m_ledBuffer.setRGB(led, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_ledStrip.setData(m_ledBuffer);
  }
}
