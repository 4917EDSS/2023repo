// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.SparkMaxSub;

public class RunSparkMaxWithJoystickCmd extends CommandBase {
  CommandPS4Controller m_controller;
  SparkMaxSub m_sparkMaxSub;

    // Use addRequirements() here to declare subsystem dependencies.
  /** Creates a new RunSparkMaxWithJoystickCmd. */
  public RunSparkMaxWithJoystickCmd(CommandPS4Controller controller, SparkMaxSub sparkMaxSub) {
    m_controller = controller;
    m_sparkMaxSub = sparkMaxSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sparkMaxSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_sparkMaxSub.setPower(m_controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sparkMaxSub.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
