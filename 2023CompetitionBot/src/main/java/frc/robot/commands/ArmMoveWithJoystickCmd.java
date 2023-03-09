// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.StateOfRobot;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.SubControl;

public class ArmMoveWithJoystickCmd extends CommandBase {
  private final CommandPS4Controller m_controller;
  private final ArmSub m_armSub;

  double powerSafety = 1.0;

  /** Creates a new ArmMoveWithJoystickCmd. */
  public ArmMoveWithJoystickCmd(CommandPS4Controller controller, ArmSub armSub) {
    m_controller = controller;
    m_armSub = armSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  private double adjustSensitivity(double power, int sensitvity) {
    double dir;
    if(power < 0) {
      dir = -1;
    } else {
      dir = 1;
    }

    power = Math.pow(Math.abs(power), sensitvity) * dir;

    return power;
  }

  /*
   * Didn't end up needing this private double slowCurve(double val, double curve, double start, double end) {
   * //y=a*c^x+b // Slow value down by a curve between (0-1) // b = end // c = curve // x = val double a = start - end;
   * return a * Math.pow(curve, val) + end; }
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powerSafety = 1.0;
    // Uncomment if needed
    /*
     * double distDanger = m_armSub.distToDanger(m_armSub.getPosition());
     * 
     * if(distDanger < maxDangerDist && (m_armSub.aboveDangerZone(distDanger)|| m_armSub.belowDangerZone(distDanger))) {
     * double powerChange = slowCurve(distDanger,0.9999,0.0,maxDangerDist);
     * 
     * powerSafety = powerChange/maxDangerDist;
     * 
     * }
     */
    if(!StateOfRobot.m_operatorJoystickforIntake) {
      m_armSub.setPosition(SubControl.Mode.MANUAL,
          MathUtil.clamp(adjustSensitivity(m_controller.getRightY(), 3), -powerSafety, powerSafety), 0.0); // Make sure arm isn't too sensitive
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSub.setPosition(SubControl.Mode.MANUAL, 0.0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
