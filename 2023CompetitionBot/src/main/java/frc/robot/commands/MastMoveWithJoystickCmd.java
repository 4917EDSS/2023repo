// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * @author Zippy
 * 
 * @author userCS45
 * 
 * @author frc4917
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MastMoveWithJoystickCmd extends CommandBase {
  private final CommandPS4Controller m_controller;
  private final MastSub m_mastSub;

  /** Creates a new MastMoveWithJoystickCmd. */
  public MastMoveWithJoystickCmd(CommandPS4Controller controller, MastSub mastSub) {
    m_controller = controller;
    m_mastSub = mastSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mastSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_mastSub.setPosition(Mode.MANUAL, -m_controller.getLeftY(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_mastSub.setPosition(Mode.MANUAL, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
