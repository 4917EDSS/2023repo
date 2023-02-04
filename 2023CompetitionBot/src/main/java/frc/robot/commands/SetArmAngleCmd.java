// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ManipulatorSub;

public class SetArmAngleCmd extends CommandBase {
  /** Creates a new SetArmAngleCmd. */
  private final CommandPS4Controller m_controller;
  private final ManipulatorSub m_manipulatorSub;
  public SetArmAngleCmd(CommandPS4Controller controller, ManipulatorSub manipulatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_manipulatorSub = manipulatorSub;
    addRequirements(manipulatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.povDown().getAsBoolean()) {
      m_manipulatorSub.setArmAngle(0.0);
    }
    else if(m_controller.povLeft().getAsBoolean()) {
      m_manipulatorSub.setArmAngle(-80.0);
    }
    else if(m_controller.povRight().getAsBoolean()) {
      m_manipulatorSub.setArmAngle(80.0);
    }
    else {
      m_manipulatorSub.rotateArm(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
