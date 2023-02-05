// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSub extends SubsystemBase {
  /** Creates a new GripperSub. */
  private final Solenoid m_solenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM,
      Constants.SolenoidIds.kGripperCylinder);

  public GripperSub() {
    m_solenoidPCM.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Use this method to reset all of the hardware and states to safe starting values */
  public void init() {
    //TODO: Add resets here.  Call from constructor.
  }

  /** This method puts the subsystem in a safe state when all commands are interrupted */
  public void interrupt() {
    // Nothing to do on interrupt at this time
  }

  public void setValve(boolean isOpen) {
    m_solenoidPCM.set(isOpen);
  }
}
