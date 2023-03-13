// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateOfRobot;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;


public class IntakeGamePieceCmd extends CommandBase {
  private double m_power;
  private double m_timeSpinning = 0.1;
  private IntakeSub m_intakeSub;
  private LedSub m_ledSub;

  /** Creates a new IntakeGamePieceCmd. */
  public IntakeGamePieceCmd(double power, IntakeSub intakeSub, LedSub ledSub) {
    m_power = power;
    m_intakeSub = intakeSub;
    m_ledSub = ledSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double curPower = m_power;
    if(StateOfRobot.isConeMode()) {
      curPower = -curPower;
    }

    m_intakeSub.spinWheelsIntake(curPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.spinWheelsIntake(0);
    if(!interrupted) {
      m_ledSub.Flash(LedSub.LedColour.GREEN);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSub.isIntakeLoaded();
    //if(RobotController.getFPGATime() - m_timeSpinning > (m_timeSpinning * Math.pow(10, 6))) { // After (timeSpinning) seconds the command stops automatically
    //return true; 
    //}
  }
}

