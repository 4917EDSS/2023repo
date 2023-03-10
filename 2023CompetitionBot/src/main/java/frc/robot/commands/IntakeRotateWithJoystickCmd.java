// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.SubControl.Mode;
import frc.robot.StateOfRobot;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.SubControl;

public class IntakeRotateWithJoystickCmd extends CommandBase {
  private final CommandPS4Controller m_controller;
  private final IntakeSub m_intakeSub;

  /** Creates a new RotateIntakeWithJoystickCmd. */
  public IntakeRotateWithJoystickCmd(CommandPS4Controller controller, IntakeSub intakeSub) {
    m_controller = controller;
    m_intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(StateOfRobot.m_operatorJoystickforIntake) {// && !m_intakeSub.inDangerZone()) {
      m_intakeSub.setPosition(SubControl.Mode.MANUAL, m_controller.getLeftY(), 0);
      //m_intakeSub.setPosition(Mode.MANUAL, -m_controller.getLeftY(), 0);
      m_intakeSub.spinWheelsIntake(-m_controller.getRightY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setPosition(Mode.MANUAL, 0, 0);
    m_intakeSub.spinWheelsIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
