// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.ManipulatorSub;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CylinderCmd extends CommandBase {
  /** Creates a new Cylinder. */
  Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIds.solenoidId);
  public CylinderCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_solenoid.set(false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setSolenoid(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void setSolenoid(boolean isOn) {
    m_solenoid.set(isOn);
  }
}
