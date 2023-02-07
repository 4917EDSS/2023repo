// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.ManipulatorSub;

public class InterruptAllCommandsCmd extends CommandBase {

  /** Creates a new InterruptAllCommandsCmd. */
  private final ManipulatorSub m_manipulatorSub;
  private final GripperSub m_gripperSub;
  private final DrivetrainSub m_drivetrainSub;

  public InterruptAllCommandsCmd(ManipulatorSub manipulatorSub, GripperSub gripperSub, DrivetrainSub drivetrainSub) {
    m_manipulatorSub = manipulatorSub;
    m_gripperSub = gripperSub;
    m_drivetrainSub = drivetrainSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSub, gripperSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorSub.interrupt();
    m_drivetrainSub.interrupt();
    m_gripperSub.interrupt();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
