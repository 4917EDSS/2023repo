// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
Hardware Info: 
https://robotics.choate.edu/wp-content/uploads/2020/01/Falcon500UserGuide-20191101.pdf
API:
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/hal/FRCNetComm.tResourceType.html#kResourceType_TalonFX
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/motorcontrol/PWMTalonFX.html
Vendor Parts > CTRE Falcon 500:
https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/ 
2048 CPR
*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFXSub extends SubsystemBase {
 TalonFX m_falcon500 = new TalonFX(1);
  /** Creates a new TalonFXSub. */
  public TalonFXSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setMotorPower(double power) {
    m_falcon500.set(ControlMode.PercentOutput, power);
  }

}
