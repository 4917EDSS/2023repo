// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.basic.BasicBorders.MarginBorder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.SubControl;
import frc.robot.subsystems.SubControl.Mode;

public class SetLimitSwitchesCmd extends CommandBase {
  private final MastSub m_mastSub;
  private final ArmSub m_armSub;
  private final IntakeSub m_intakeSub;

  private int m_armPhase = 0;

  private double kArmLimitPos = -19000.0;

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
  public void initialize() {
    m_intakeSub.setPosition(Mode.DISABLED, 0, 0);
    m_mastSub.setPosition(Mode.DISABLED, 0, 0);
    //TODO add arm
    m_armSub.setPosition(Mode.DISABLED, 0, 0);

    m_armPhase = 0;
  }

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

    // Arm logic
    switch(m_armPhase) {
      case 0: // Find relative zero
        m_armSub.move(-0.3);
        if(m_armSub.isArmAtSwitch()) {
          m_armSub.setEncoderPosition(kArmLimitPos);
          m_armPhase = 1;
        }
        break;

      case 1: // Go past zero a bit
        m_armSub.move(-0.3);
        if(m_armSub.getPosition() < kArmLimitPos - 4000.0) {
          m_armPhase = 2;
        }
        break;

      case 2: // Carefully go to zero position
        if(!m_armSub.isArmAtSwitch()) {
          m_armSub.move(0.2);
        } else {
          m_armSub.move(0.0);
          m_armPhase = 3;
        }
        break;
    }

    //TODO add arm 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_mastSub.zeroEncoder();
    m_armSub.setEncoderPosition(kArmLimitPos);
    m_intakeSub.zeroEncoderRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_mastSub.isMastAtLimit() && m_intakeSub.isIntakeAtLimit() && (m_armSub.isArmAtSwitch() && m_armPhase == 3)) {
      return true;
    }
    return false;
  }
  //TODO add arm 
}
