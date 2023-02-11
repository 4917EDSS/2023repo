// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  private final int kLedStripPwmPort = 1;
  private final int kLedStripLength = 10;
  private final int kLedStripDefaultRed = 250;
  private final int kLedStripDefaultGreen = 0;
  private final int kLedStripDefaultBlue = 250;

  private final AddressableLED m_ledStrip = new AddressableLED(kLedStripPwmPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);
  long tick = System.currentTimeMillis();

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_ledStrip.setLength(kLedStripLength);

    setColor(-1, kLedStripDefaultRed, kLedStripDefaultGreen, kLedStripDefaultBlue);
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(System.currentTimeMillis() - tick >= 200) {
      tick = System.currentTimeMillis();
      int r = (int) (Math.random() * 255);
      int g = (int) (Math.random() * 255);
      int b = (int) (Math.random() * 255);

      setColor(-1, r, g, b);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

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

    if(i < 0) {
      for(int led = 0; led < kLedStripLength; led++) {
        m_ledBuffer.setRGB(led, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_ledStrip.setData(m_ledBuffer);
  }

}
