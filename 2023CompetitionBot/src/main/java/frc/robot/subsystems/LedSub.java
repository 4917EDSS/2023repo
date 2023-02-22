// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.PwmIds;
import frc.robot.Constants.LedConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */


public class LedSub extends SubsystemBase {
  //previous RGB colour vars
  private int m_rr, m_gg, m_bb = 0;


  public enum LedZones {
    ZONE0(0, 2), ZONE1(3, 5), ZONE3(6, 9);

    public final int start;
    public final int end;

    LedZones(int start, int end) {
      this.start = start;
      this.end = end;
    }
  }

  public enum LedColour {
    YELLOW(128, 128, 0), PURPLE(80, 20, 60), RED(128, 0, 0), GREEN(0, 128, 0), BLUE(0, 0, 128), WHITE(128, 128, 128);

    public final int red, blue, green;

    LedColour(int red, int green, int blue) {
      int r = red;
      int g = green;
      int b = blue;

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

      this.red = r;
      this.green = g;
      this.blue = b;
      
    }
  }

  // Hardware setup.
  AddressableLED m_ledStrip = new AddressableLED(PwmIds.kLedStripPwmPort);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedConstants.kLedStripLength);

  /** Creates a new LedSub. */
  public LedSub() {
    m_ledStrip.setLength(m_ledBuffer.getLength());
    setColor(-1, LedColour.WHITE);
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();

    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Use this method to reset all of the hardware and states to safe starting values
   */
  public void init() {
    setColor(-1, LedColour.WHITE);
  }


  /**
   * This method puts the subsystem in a safe state when all commands are interrupted
   */
  public void interrupt() {

  }

  public void setZoneColour(LedZones zone, LedColour ledColour) {

    for(int i = zone.start; i <= zone.end; i++) {
      setLedBuffer(i, ledColour);
    }
    m_ledStrip.setData(m_ledBuffer);
  }

  public void setColor(int i, LedColour ledColour) {
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
 
    setLedBuffer(i, ledColour);
    if(i < 0) {
      m_ledStrip.setData(m_ledBuffer);
    }
  }

  // LED state enums
  public enum LEDMode {
    ConeMode, CubeMode,

  }

  public void setLedBuffer(int i, LedColour ledColour) {
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
    //
    if(i < 0) {

      for(int led = 0; led < LedConstants.kLedStripLength; led++) {
        //Light string takes BRG
        m_ledBuffer.setRGB(led, ledColour.red,ledColour.blue,  ledColour.green);
      }
    } else {
      //Light string takes BRG
      m_ledBuffer.setRGB(i, ledColour.red, ledColour.blue, ledColour.green);
    }
  }

  //Set Colours according to state
  public void setLEDState(LEDMode LEDState) {
    if(LEDState == LEDMode.ConeMode) {
      setColor(0, LedColour.YELLOW); //Set LED colour to yellow 
    } else if(LEDState == LEDMode.CubeMode) {
      setColor(0, LedColour.PURPLE); //Set LED colour to purple
    }
  }

  public void smartDashboardSet() {

    // double[] colour = {Double.valueOf(m_rr), Double.valueOf(m_gg), Double.valueOf(m_bb)};

    // SmartDashboard.putNumberArray("null", colour);


  }

}
