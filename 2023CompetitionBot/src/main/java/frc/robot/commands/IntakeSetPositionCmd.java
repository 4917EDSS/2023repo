// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateOfRobot;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ManipulatorsPositions;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;

public class IntakeSetPositionCmd extends CommandBase {

  public final static int kSquareButton = 0;
  public final static int kOptionsButton = 1;

  private ManipulatorsPositions m_manipulatorsPositions;
  private final static double kMaxArmPower = 1.0;
  private final static double kMaxMastPower = 1.0;
  private final static double kMaxIntakePower = 0.75;
  private long m_timeStart; // Time constraint incase it never aligns

  //private final ManipulatorSub m_manipulatorSub;
  private final ArmSub m_armSub;
  private final MastSub m_mastSub;
  private final IntakeSub m_intakeSub;

  /** Creates a new MoveManipulatorToHighPickUpCmd. */
  public IntakeSetPositionCmd(ManipulatorsPositions positions, ArmSub armSub, MastSub mastSub, IntakeSub intakeSub) {
    m_armSub = armSub;
    m_mastSub = mastSub;
    m_manipulatorsPositions = positions;
    m_intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub, mastSub, intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ManipulatorsPositions converted = ManipulatorsPositions.convert(m_manipulatorsPositions, StateOfRobot.isConeMode());

    m_armSub.setPosition(Mode.AUTO, kMaxArmPower, converted.armEncoder);
    m_mastSub.setPosition(Mode.AUTO, kMaxMastPower, converted.mastEncoder);
    m_intakeSub.setPosition(Mode.AUTO, kMaxIntakePower, converted.wristEncoder);
    if(StateOfRobot.isConeMode()) {
      m_intakeSub.spinWheelsIntake(-0.22);
    }
    m_timeStart = RobotController.getFPGATime();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotController.getFPGATime() - m_timeStart > 250000) {
      m_intakeSub.spinWheelsIntake(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.spinWheelsIntake(0);
    // if inturrupted, stop otherwise already stopped
    if(interrupted) {
      m_armSub.setPosition(Mode.MANUAL, 0, 0);
      m_mastSub.setPosition(Mode.MANUAL, 0, 0);
      m_intakeSub.setPosition(Mode.MANUAL, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotController.getFPGATime() - m_timeStart > 2500000) { // After 1.5 seconds the command stops automatically
      System.out.println("**** timed out** intake set position");
      return true;
    }
    return (m_armSub.isFinished() && m_mastSub.isFinished() && m_intakeSub.isFinished());
  }
}
