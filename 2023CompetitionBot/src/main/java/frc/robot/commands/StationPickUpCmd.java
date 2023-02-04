// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSub;

public class StationPickUpCmd extends CommandBase {
  private static final double kStationArmAngle = 55;
  private static final double kStationMastPosition = 0;
  
  private final ManipulatorSub m_manipulatorSub;
  /** Creates a new MoveManipulatorToHighPickUpCmd. */
  public StationPickUpCmd(ManipulatorSub manipulatorSub) {
    m_manipulatorSub = manipulatorSub;
    addRequirements(manipulatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSub.setArmAngle(kStationArmAngle);
    m_manipulatorSub.setMastPosition(kStationMastPosition);
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

    if((m_manipulatorSub.getArmAngle() >= (kStationArmAngle - 5)) && (m_manipulatorSub.getArmAngle() <= (kStationArmAngle +5))){
      isArmAngleInPosition = true;
    }
    if((m_manipulatorSub.getMastPosition() >= (kStationMastPosition - 5)) && (m_manipulatorSub.getMastPosition() <= (kStationMastPosition + 5))){
      isMastInPosition = true;
    }

    return(isArmAngleInPosition && isMastInPosition); 
  }
}
