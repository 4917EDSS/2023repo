// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());




  // Create the drivetrain motors both left and right for SparkMax encoder
  private final CANSparkMax m_leftBackMotor = 
      new CANSparkMax(Constants.CanIds.kLeftBackDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightBackMotor = 
      new CANSparkMax(Constants.CanIds.kRightBackDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private final CANSparkMax m_leftFrontMotor = 
      new CANSparkMax(Constants.CanIds.kLeftFrontDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = 
      new CANSparkMax(Constants.CanIds.kRightFrontDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  public DrivetrainSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftPower, double rightPower) {
    // m_leftMotor.set(leftPower);
    // m_rightMotor.set(-rightPower);
  }
}
