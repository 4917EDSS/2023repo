// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ManipulatorSub extends SubsystemBase {
  
  private final CANSparkMax m_armMotor = 
      new CANSparkMax(Constants.CanIds.kRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new TemplateSub. */
  public ManipulatorSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void rotateArm(double armPower) {
    m_armMotor.set(armPower);
  }
  public void tankDrive(double a, double b){}
}
