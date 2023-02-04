// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.ManipulatorSub;

public class KillSwitchCmd extends CommandBase {

  /** Creates a new KillSwitchCmd. */
  private final ManipulatorSub m_manipulatorSub;
  private final GripperSub m_gripperSub;
  private final DrivetrainSub m_drivetrainSub;
  public KillSwitchCmd(ManipulatorSub manipulatorSub, GripperSub gripperSub, DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
   m_manipulatorSub = manipulatorSub;
   m_gripperSub = gripperSub; 
   m_drivetrainSub = drivetrainSub; 
    addRequirements(manipulatorSub, gripperSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorSub.kill();
    m_drivetrainSub.kill();
    m_gripperSub.Kill();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_manipulatorSub.rotateArm(0.0); // Set arm's power to 0
    // m_manipulatorSub.moveMast(0.0); // Set arm's power to 0
    // m_gripperSub.setValve(true); // Set solenoid to open, We will not do anything to the solenoid
    // m_drivetrainSub.tankDrive(0.0,0.0); // Set tank drivetrain to 0 Keep or remove?
    // m_drivetrainSub.arcadeDrive(0.0, 0.0); // Set arcade drivetrain to 0
    //TODO: Add gripper motor if implemented
    
   

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return true;  // This needs to be false to allow command to run indefinately.
  }
}
