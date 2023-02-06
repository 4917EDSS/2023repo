// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSub;

public class PickUpCmd extends CommandBase {
  private final static double kStationArmAngle = -45;
  private final static double kStationMastPosition = 13;
  private final static double kGroundArmAngle = -30;
  private final static double kGroundMastPosition = 30;

  public final static int kSquareButton = 0;
  public final static int kOptionsButton = 1;

  private double m_armAngle = 0;
  private double m_mastPosition = 0;

  private final ManipulatorSub m_manipulatorSub;

  /** Creates a new MoveManipulatorToHighPickUpCmd. */
  public PickUpCmd(ManipulatorSub manipulatorSub, int button) {
    m_manipulatorSub = manipulatorSub;
    addRequirements(manipulatorSub);

    if(button == kSquareButton) {
      m_armAngle = kStationArmAngle;
      m_mastPosition = kStationMastPosition;
    } else if(button == kOptionsButton) {
      m_armAngle = kGroundArmAngle;
      m_mastPosition = kGroundMastPosition;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSub.setArmAngle(m_armAngle);
    m_manipulatorSub.setMastPosition(m_mastPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulatorSub.rotateArm(0); // Stops the arm once it reaches the position
    m_manipulatorSub.moveMast(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isArmAngleInPosition = false;
    boolean isMastInPosition = false;

    if((m_manipulatorSub.getArmAngle() >= (m_armAngle - 5)) && (m_manipulatorSub.getArmAngle() <= (m_armAngle + 5))) {
      isArmAngleInPosition = true;
    }
    if((m_manipulatorSub.getMastPosition() >= (m_mastPosition - 5))
        && (m_manipulatorSub.getMastPosition() <= (m_mastPosition + 5))) {
      isMastInPosition = true;
    }

    return (isArmAngleInPosition && isMastInPosition);
  }
}
