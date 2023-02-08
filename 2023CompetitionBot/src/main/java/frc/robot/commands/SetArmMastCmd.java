// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ManipulatorSub;

public class SetArmMastCmd extends CommandBase {
  /** Creates a new SetArmMastCmd. */
  private final CommandPS4Controller m_operatorController;
  private final ManipulatorSub m_manipulatorSub;

  public SetArmMastCmd(CommandPS4Controller controller, ManipulatorSub manipulatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_operatorController = controller;
    m_manipulatorSub = manipulatorSub;
    addRequirements(manipulatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_operatorController.povUp().getAsBoolean()) { // if manually controlled, this command won't run
      m_manipulatorSub.setArmAngle(0.0);
      m_manipulatorSub.autoSetMastPosition(30.0);
    } else {
      if(m_operatorController.share().getAsBoolean()) {
        if(m_operatorController.povDown().getAsBoolean()) {
          m_manipulatorSub.setArmAngle(0.0);
        } else if(m_operatorController.povLeft().getAsBoolean()) {
          m_manipulatorSub.setArmAngle(-80.0);
        } else if(m_operatorController.povRight().getAsBoolean()) {
          m_manipulatorSub.setArmAngle(80.0);
        } else {
          m_manipulatorSub.moveMast(0.0);
          m_manipulatorSub.rotateArm(0.0);
        }
      } else {
        if(m_operatorController.povDown().getAsBoolean()) {
          m_manipulatorSub.autoSetMastPosition(30.0);
        } else if(m_operatorController.povLeft().getAsBoolean()) {
          m_manipulatorSub.autoSetMastPosition(5.0);
        } else if(m_operatorController.povRight().getAsBoolean()) {
          m_manipulatorSub.autoSetMastPosition(55.0);
        } else {
          m_manipulatorSub.moveMast(0.0);
          m_manipulatorSub.rotateArm(0.0);
        }
      }
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
