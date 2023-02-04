// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LedConstants;

public class LedSub extends SubsystemBase {
  // Hardware setup.
  AddressableLED m_ledStrip = new AddressableLED(LedConstants.kLedStripPwmPort);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedConstants.kLedStripLength);

  /** Creates a new LedSub. */
  public LedSub() {
    m_ledStrip.setLength(LedConstants.kLedStripLength);
    setColor(-1, LedConstants.kLedStripDefaultRed, LedConstants.kLedStripDefaultGreen,
        LedConstants.kLedStripDefaultBlue);
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    if (r < 0) {
      r = 0;
    }
    if (r > 255) {
      r = 255;
    }
    if (g < 0) {
      g = 0;
    }
    if (g > 255) {
      g = 255;
    }
    if (b < 0) {
      b = 0;
    }
    if (b > 255) {
      b = 255;
    }
    if (i < 0) {
      for (int led = 0; i < LedConstants.kLedStripLength; led++) {
        m_ledBuffer.setRGB(led, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    
    m_ledStrip.setData(m_ledBuffer);
  }

  // LED state enums
  public enum LEDMode {
    ConeMode,           
    CubeMode,
    
  }
//Set Colours according to state
  public void setLEDState (LEDMode LEDState){
    if (LEDState == LEDMode.ConeMode) {
      setColor(-1, 255, 255, 0);         //Set LED colour to yellow 
    }
    else if (LEDState == LEDMode.CubeMode) {
      setColor(-1, 230, 230, 250);       //Set LED colour to purple
    }
  }
}