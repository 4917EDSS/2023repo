// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArmSub;

public class ArmWithJoystickCmd extends CommandBase {
  boolean m_enable;
  CommandPS4Controller m_controller;
  ArmSub m_armSub;
  int m_loopCount;

  /** Creates a new ArmWithJoystickCmd. */
  public ArmWithJoystickCmd(boolean enable, CommandPS4Controller controller, ArmSub armSub) {    
    m_enable = enable;
    m_controller = controller;
    m_armSub = armSub;
    m_loopCount = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_enable) {
      if(++m_loopCount == 10) {
        m_loopCount = 0;
        System.out.println("Arm " + m_controller.getLeftY());
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
