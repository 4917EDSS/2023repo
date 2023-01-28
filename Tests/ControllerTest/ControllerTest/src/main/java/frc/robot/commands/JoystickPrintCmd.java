// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.JoystickSub;


public class JoystickPrintCmd extends CommandBase {

  int printTimer = 0;

  private final CommandPS4Controller m_controller;

  /** Creates a new JoystickPrintCmd. */
  public JoystickPrintCmd(CommandPS4Controller controller, JoystickSub joystickSub) {
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(joystickSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    printTimer += 1;

    if(printTimer == 10)
    {
      if(m_controller.getLeftX() != 0)
      {
        System.out.print("LX ");
        System.out.println(m_controller.getLeftX());
      }

      printTimer = 0;
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
