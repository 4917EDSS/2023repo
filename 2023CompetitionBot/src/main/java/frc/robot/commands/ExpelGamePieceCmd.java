// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.IntakeSub;

public class ExpelGamePieceCmd extends CommandBase {
  /** Creates a new ExpelGamePieceCmd. */

  private final IntakeSub m_intakeSub;
  private long m_timeStart; // The time it was at the time the command was initialized
  private double timeSpinning = 0.2; // How long the intake wheels spin (in seconds)
  private double power = 1; // What power the motors spin at

  public ExpelGamePieceCmd(IntakeSub intakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSub = intakeSub;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeStart = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.spinWheelsIntake(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.spinWheelsIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(RobotController.getFPGATime() - m_timeStart > (timeSpinning * Math.pow(10, 6))) { // After (timeSpinning) seconds the command stops automatically
      return true;
    }

    return false;
  }
}
