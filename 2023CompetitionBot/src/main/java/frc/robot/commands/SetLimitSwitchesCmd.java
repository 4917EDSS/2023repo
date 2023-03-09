// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MastSub;

public class SetLimitSwitchesCmd extends CommandBase {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;

  /** Creates a new SetLimitSwitchesCmd. */
  public SetLimitSwitchesCmd(MastSub mastSub, ArmSub armSub, IntakeSub intakeSub) {
    m_mastSub = mastSub;
    m_armSub = armSub;
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mastSub);
    addRequirements(armSub);
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_mastSub.isMastAtLimit()) {
      m_mastSub.move(-0.4);
    } else {
      m_mastSub.move(0);
    }

    if(!m_intakeSub.isIntakeAtLimit()) {
      m_intakeSub.intakeRotate(-0.3);
    } else {
      m_intakeSub.intakeRotate(0);
    }
    //TODO add arm 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_mastSub.zeroEncoder();
    // m_armSub.zeroEncoder(); // TODO add with arm
    m_intakeSub.zeroEncoderRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_mastSub.isMastAtLimit() && m_intakeSub.isIntakeAtLimit()) {
      return true;
    }
    return false;
  }
  //TODO add arm 
}
