// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl.Mode;

public class PickUpCmd extends CommandBase {
  private final static double kStationArmPosition = -45;
  private final static double kStationMastPosition = 13;
  private final static double kGroundArmPosition = -30;
  private final static double kGroundMastPosition = 30;

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
  public PickUpCmd(ArmSub armSub, MastSub mastSub, int button) {
    m_armSub = armSub;
    m_mastSub = mastSub;

    // TODO: Remove this once we get the positions via parameters
    if(button == kSquareButton) {
      m_armPosition = kStationArmPosition;
      m_mastPosition = kStationMastPosition;
    } else if(button == kOptionsButton) {
      m_armPosition = kGroundArmPosition;
      m_mastPosition = kGroundMastPosition;
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub, mastSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:  The mode here should be AUTO since we're asking the state machine to move the mechanisms to a specific location.  MANUAL is for joystick input.
    m_armSub.setPosition(Mode.MANUAL, kMaxArmPower, m_armPosition);
    m_mastSub.setPosition(Mode.MANUAL, kMaxMastPower, m_mastPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:  Fix this code. Right now, on command end, it will set the mast and arms to position 0 at maximum possible speed!
    //Also, only need to stop the motion if we were interrupt.  If it ended properly, it would be stopped already
    // m_armSub.setPosition(Mode.MANUAL, kMaxArmPower, 0);
    // m_mastSub.setPosition(Mode.MANUAL, kMaxMastPower, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_armSub.isFinished() && m_mastSub.isFinished());
  }
}
