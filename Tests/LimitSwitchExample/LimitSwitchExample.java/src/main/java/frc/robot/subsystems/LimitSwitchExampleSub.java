// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
/**
 Hardware Info:
 Limit swicthes connect to a DIO port and are fairly simple to connect, when connected to the DIO port, it will read "high" if the
 circuit is open and "low" if it is closed. Switches connect between the Signal and Ground pin on the DIO port.
 */
public class LimitSwitchExampleSub extends SubsystemBase {
  private final DigitalInput toplimitSwitch = new DigitalInput(0);

  /** Creates a new LimitSwitchExampleSub. */
  public LimitSwitchExampleSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isSwitchPressed () {
    return toplimitSwitch.get();
  }

}






