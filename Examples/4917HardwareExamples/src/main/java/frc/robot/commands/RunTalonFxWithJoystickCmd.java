// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.TalonFxSub;

public class RunTalonFxWithJoystickCmd extends CommandBase {
  CommandPS4Controller m_controller;
  TalonFxSub m_talonFxSub;

  /** Creates a new RunTalonFxWithJoystickCmd. */
  public RunTalonFxWithJoystickCmd(CommandPS4Controller controller, TalonFxSub talonFxSub) {
    m_controller = controller;
    m_talonFxSub = talonFxSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonFxSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_talonFxSub.setPower(m_controller.getLeftX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_talonFxSub.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
