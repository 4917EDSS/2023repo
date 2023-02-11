// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.SubControl;

public class ArmMoveWithJoystickCmd extends CommandBase {
  private final CommandPS4Controller m_controller;
  private final ArmSub m_armSub;

  /** Creates a new ArmMoveWithJoystickCmd. */
  public ArmMoveWithJoystickCmd(CommandPS4Controller controller, ArmSub armSub) {
    m_controller = controller;
    m_armSub = armSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSub.setPosition(SubControl.Mode.MANUAL, m_controller.getRightY(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSub.setPosition(SubControl.Mode.MANUAL, 0.0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}