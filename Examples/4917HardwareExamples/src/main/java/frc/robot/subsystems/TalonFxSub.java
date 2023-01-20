// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Hardware Info:
 * https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
 * https://robotics.choate.edu/wp-content/uploads/2020/01/Falcon500UserGuide-20191101.pdf
 * 
 * API:
 * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/hal/FRCNetComm.tResourceType.html#kResourceType_TalonFX
 * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/motorcontrol/PWMTalonFX.html
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonFxSub extends SubsystemBase {
  // The ID is the CAN ID that the motor was programmed with (using the CTRE Phoenix software)
  TalonFX m_motor = new TalonFX(Constants.CanIds.talonFxId);

  /** Creates a new TalonFxSub. */
  public TalonFxSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    m_motor.set(ControlMode.PercentOutput, power);
  }
}
