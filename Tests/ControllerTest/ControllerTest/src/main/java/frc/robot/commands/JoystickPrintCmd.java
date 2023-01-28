// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class JoystickPrintCmd extends CommandBase {

  private final CommandPS4Controller m_driverController = 
  new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** Creates a new JoystickPrintCmd. */
  public JoystickPrintCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_driverController.axisGreaterThan(0, 0.1).onTrue(new PrintCommand("Axis 0 greater than 0"));
    m_driverController.axisGreaterThan(1, 0.1).onTrue(new PrintCommand("Axis 1 greater than 0"));
    m_driverController.axisGreaterThan(2, 0.1).onTrue(new PrintCommand("Axis 2 greater than 0"));
    m_driverController.axisGreaterThan(3, 0.1).onTrue(new PrintCommand("Axis 3 greater than 0"));

    m_driverController.axisLessThan(0, 0.1).onTrue(new PrintCommand("Axis 0 less than 0"));
    m_driverController.axisLessThan(1, 0.1).onTrue(new PrintCommand("Axis 1 less than 0"));
    m_driverController.axisLessThan(2, 0.1).onTrue(new PrintCommand("Axis 2 less than 0"));
    m_driverController.axisLessThan(3, 0.1).onTrue(new PrintCommand("Axis 3 less than 0"));
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
