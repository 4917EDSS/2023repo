// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ManipulatorSub;

public class RotateArmWithJoystickCmd extends CommandBase {
  private final CommandPS4Controller m_controller;
  private final ManipulatorSub m_manipulatorSub;

  /** Creates a new RotateArmWithJoystickCmd. */
  public RotateArmWithJoystickCmd(CommandPS4Controller controller, ManipulatorSub manipulatorSub) {
    m_controller = controller;
    m_manipulatorSub = manipulatorSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSub.rotateArm(m_controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulatorSub.rotateArm(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
