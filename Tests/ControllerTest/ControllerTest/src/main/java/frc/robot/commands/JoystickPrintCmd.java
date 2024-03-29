// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.JoystickSub;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;


public class JoystickPrintCmd extends CommandBase {

  private int count = 0; 
  
  private final JoystickSub m_JoystickSub;

  private final CommandPS4Controller m_driverController;
  //new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** Creates a new JoystickPrintCmd. */
  public JoystickPrintCmd(JoystickSub joystickSub, CommandPS4Controller driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_JoystickSub = joystickSub;
    m_driverController = driverController;

    addRequirements(joystickSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    count++;
    
    if(m_driverController.getLeftX() < -0.2f && count >= 25 || m_driverController.getLeftX() > 0.2f && count >= 25)
    {
      System.out.print("Left X Axis: ");
      System.out.println(m_driverController.getLeftX());
    }

    if(m_driverController.getLeftY() < -0.2f && count >= 25 || m_driverController.getLeftY() > 0.2f && count >= 25)
    {
      System.out.print("Left Y Axis: ");
      System.out.println(m_driverController.getLeftY());
    }

    if(m_driverController.getRightX() < -0.2f && count >= 25 || m_driverController.getRightX() > 0.2f && count >= 25)
    {
      System.out.print("Right X Axis: ");
      System.out.println(m_driverController.getRightX());
    }

    if(m_driverController.getRightY() < -0.2f && count >= 25 || m_driverController.getRightY() > 0.2f && count >= 25)
   {
      System.out.print("Right Y Axis: ");
      System.out.println(m_driverController.getRightY());
    }

    if(count >= 25)
    {
      count = 0;
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
