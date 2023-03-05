// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.PwmIds;
import frc.robot.Constants.LedConstants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */


public class LedSub extends SubsystemBase {
  //previous RGB colour vars

  //  private int m_rr, m_gg, m_bb = 0;


  public enum LedZones {
    DIAG_MAST_ENC(0, 0), DIAG_ARM_ENC(1, 1), DIAG_INTAKE_LIMSWITCH(2, 2), ZONE0(0, 2), ZONE1(3, 5), ZONE2(6, 9), ALL(0,
        9);

    public final int start;
    public final int end;

    LedZones(int start, int end) {
      this.start = start;
      this.end = end;
    }
  }

  public enum LedColour {
    YELLOW(128, 128, 0), PURPLE(80, 20, 60), RED(128, 0, 0), GREEN(0, 128, 0), START_GREEN(0, 64, 0), BLUE(0, 0,
        128), WHITE(128, 128, 128);

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
    setZoneColour(LedZones.ALL, LedColour.START_GREEN);
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
    setZoneColour(LedZones.ALL, LedColour.START_GREEN);
  }


  /**
   * This method puts the subsystem in a safe state when all commands are interrupted
   */
  public void interrupt() {

  }

  public void setZoneColour(LedZones zone, LedColour ledColour) {
    setZoneRGB(zone, ledColour.red, ledColour.green, ledColour.blue);
  }

  public void setZoneRGB(LedZones zone, int r, int g, int b) {
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
    for(int i = zone.start; i <= zone.end; i++) {
      m_ledBuffer.setRGB(i, r, b, g); //this function takes in RBG
    }
    m_ledStrip.setData(m_ledBuffer);
  }

  // LED state enums
  public enum LEDMode {
    ConeMode, CubeMode,
  }

  //Set Colours according to state
  public void setLEDState(LEDMode LEDState) {
    if(LEDState == LEDMode.ConeMode) {
      setZoneColour(LedZones.ZONE0, LedColour.YELLOW); //Set LED colour to yellow 
    } else if(LEDState == LEDMode.CubeMode) {
      setZoneColour(LedZones.ZONE0, LedColour.PURPLE); //Set LED colour to purple
    }
  }

  public void smartDashboardSet() {

    // double[] colour = {Double.valueOf(m_rr), Double.valueOf(m_gg), Double.valueOf(m_bb)};

    // SmartDashboard.putNumberArray("null", colour);


  }

}
