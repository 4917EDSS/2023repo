// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;

public class PickUpCmd extends CommandBase {
  private final static double kStationArmAngle = -45;
  private final static double kStationMastPosition = 13;
  private final static double kGroundArmAngle = -30;
  private final static double kGroundMastPosition = 30;

  public final static int kSquareButton = 0;
  public final static int kOptionsButton = 1;

  private double m_armAngle = 0;
  private double m_mastPosition = 0;
  private final static double kMaxArmPower = 0.5; // <----- Tune this value 
  private final static double kMaxMastPower = 0.5; // <----- Tune this value 

  //private final ManipulatorSub m_manipulatorSub;
  private final ArmSub m_armSub;
  private final MastSub m_mastSub;

  /** Creates a new MoveManipulatorToHighPickUpCmd. */
  public PickUpCmd(ArmSub armSub, MastSub mastSub, int button) {
    m_armSub = armSub;
    m_mastSub = mastSub;
    addRequirements(armSub,mastSub);

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
    m_armSub.setPosition(Mode.MANUAL,kMaxArmPower, m_armAngle); 
    m_mastSub.setPosition(Mode.MANUAL, kMaxMastPower, m_mastPosition);
    //m_manipulatorSub.autoSetMastPosition(m_mastPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSub.setPosition(Mode.MANUAL,kMaxArmPower, 0); 
    m_mastSub.setPosition(Mode.MANUAL, kMaxMastPower, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isArmAngleInPosition = false;
    boolean isMastInPosition = false;

    if((m_armSub.getPosition() >= (m_armAngle - 5)) && (m_armSub.getPosition() <= (m_armAngle + 5))) {
      isArmAngleInPosition = true;
    }
    if((m_mastSub.getPosition() >= (m_mastPosition - 5))
        && (m_mastSub.getPosition() <= (m_mastPosition + 5))) {
      isMastInPosition = true;
    }

    return (isArmAngleInPosition && isMastInPosition);
  }
}
