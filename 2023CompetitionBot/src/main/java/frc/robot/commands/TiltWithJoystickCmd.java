// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ManipulatorSub;

public class TiltWithJoystickCmd extends CommandBase {

  private final ManipulatorSub m_manipulatorSub;
  private final CommandPS4Controller m_controller;

  /** Creates a new TiltWithJoystickCmd. */
  public TiltWithJoystickCmd(CommandPS4Controller controller, ManipulatorSub manipulatorSub) {
    
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulatorSub = new ManipulatorSub();
    addRequirements(m_manipulatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSub.tilt(m_controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop motor
    m_manipulatorSub.tilt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
