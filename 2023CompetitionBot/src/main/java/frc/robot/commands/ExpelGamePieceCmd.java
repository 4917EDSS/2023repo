// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.StateOfRobot;
import frc.robot.subsystems.IntakeSub;

public class ExpelGamePieceCmd extends CommandBase {
  private final IntakeSub m_intakeSub;
  private long m_timeStart; // The time it was at the time the command was initialized
  private double m_timeSpinning = 0.5; // How long the intake wheels spin (in seconds)
  private double m_power; // What power the motors spin at

  /** Creates a new ExpelGamePieceCmd. */
  public ExpelGamePieceCmd(double power, IntakeSub intakeSub) {
    m_power = power;
    m_intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double curPower = m_power;
    if(StateOfRobot.isCubeMode()) {
      curPower = -curPower;
    }

    m_timeStart = RobotController.getFPGATime();
    m_intakeSub.spinWheelsIntake(curPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.spinWheelsIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(RobotController.getFPGATime() - m_timeStart > (m_timeSpinning * Math.pow(10, 6))) { // After (timeSpinning) seconds the command stops automatically
      return true;
    }

    return false;
  }
}
