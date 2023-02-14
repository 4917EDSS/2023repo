// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakePositions;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;

public class IntakeSetPositionCmd extends CommandBase {

  public final static int kSquareButton = 0;
  public final static int kOptionsButton = 1;

  private double m_armPosition = 0;
  private double m_mastPosition = 0;
  private final static double kMaxArmPower = 0.5; //TODO <----- Tune this value 
  private final static double kMaxMastPower = 0.5; //TODO <----- Tune this value 

  //private final ManipulatorSub m_manipulatorSub;
  private final ArmSub m_armSub;
  private final MastSub m_mastSub;

  /** Creates a new MoveManipulatorToHighPickUpCmd. */
  // TODO:  Pass in the desired mast and arm position instead of a controller button
  public IntakeSetPositionCmd(IntakePositions positions, ArmSub armSub, MastSub mastSub) {
    m_armSub = armSub;
    m_mastSub = mastSub;

      m_armPosition = positions.armEncoder;
      m_mastPosition = positions.mastEncoder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub, mastSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSub.setPosition(Mode.AUTO, kMaxArmPower, m_armPosition);
    m_mastSub.setPosition(Mode.AUTO, kMaxMastPower, m_mastPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if inturrupted, stop otherwise already stopped
    if (interrupted) {
      m_armSub.setPosition(Mode.MANUAL, 0, 0);
      m_mastSub.setPosition(Mode.MANUAL, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_armSub.isFinished() && m_mastSub.isFinished());
  }
}
