// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
Hardware Info:
Limit swicthes connect to a DIO port and are fairly simple to connect.  When connected to the DIO 
port, it will read "high" if the circuit is open and "low" if it is closed. Switches connect between 
the Signal and Ground pin on the DIO port.
See https://docs.wpilib.org/en/latest/docs/hardware/sensors/digital-inputs-hardware.html#connecting-a-simple-switch-to-a-dio-port

API Info:
https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/digital-inputs-software.html
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimitSwitchSub extends SubsystemBase {
  // The channel number specifies which DIO on the robotRio that the switch is connected to
  private final DigitalInput m_limitSwitch = new DigitalInput(Constants.DioIds.limitSwtichId);

  /** Creates a new LimitSwitchSub. */
  public LimitSwitchSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isSwitchPressed () {
    return m_limitSwitch.get();
  }
}
