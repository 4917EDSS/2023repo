// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;

public class DriveWithJoystickCmd extends CommandBase {
  private final int kForwardSensitivityPower = 2;
  private final int kTurnSensitivityPower = 1;
  private final double kDeadband = 0.0075;
  private final double kForwardMaxAcceleration = 0.1;
  private final double kTurnMaxAcceleration = 0.2;

  private final CommandPS4Controller m_controller;
  private final DrivetrainSub m_drivetrainSub;
  private double m_curFwdPower;
  private double m_curTurnPower;

  /** Creates a new DriveWithJoystickCmd. */
  public DriveWithJoystickCmd(CommandPS4Controller controller, DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_drivetrainSub = drivetrainSub;

    addRequirements(drivetrainSub);
  }

  private double adjustSensativity(double power, int sensitvity) {
    double dir;
    if(power < 0) {
      dir = -1;
    } else {
      dir = 1;
    }

    power = Math.pow(Math.abs(power), sensitvity) * dir;

    return power;
  }

  private double applyDeadband(double power) {
    if(Math.abs(power) <= kDeadband) {
      power = 0.0;
    }
    return power;
  }

  private double capAcceleration(double targetPower, double curPower, double maxAccleration) {
    boolean positiveAcceleration;
    double newPower = targetPower;
    if(curPower - targetPower < 0) {
      positiveAcceleration = true;
    } else {
      positiveAcceleration = false;
    }
    if(Math.abs(targetPower - curPower) > maxAccleration) {
      if(positiveAcceleration) {
        newPower = curPower + maxAccleration;
      } else {
        newPower = curPower - maxAccleration;
      }
    }
    return newPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.setBrakeCmd(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double fwdPower = m_controller.getLeftY();
    double turnPower = m_controller.getRightX();

    fwdPower = adjustSensativity(fwdPower, kForwardSensitivityPower);
    turnPower = adjustSensativity(turnPower, kTurnSensitivityPower);

    fwdPower = capAcceleration(fwdPower, m_curFwdPower, kForwardMaxAcceleration);
    turnPower = capAcceleration(turnPower, m_curTurnPower, kTurnMaxAcceleration);

    fwdPower = applyDeadband(fwdPower);
    turnPower = applyDeadband(turnPower);

    m_drivetrainSub.arcadeDrive(fwdPower, turnPower);
    m_curFwdPower = fwdPower;
    m_curTurnPower = turnPower;


    // Implemented auto-shifting here
    m_drivetrainSub.autoShift();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrainSub.tankDrive(0.0, 0.0); // Dont end or it will break the motors
    m_drivetrainSub.setBrakeCmd(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
