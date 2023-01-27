// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSub extends SubsystemBase {
  private static final double kMastEncoderMax = 60;
  private static final double kMastEncoderMin = 0;

  private double m_mastPower;

  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.CanIds.kArmMotor,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CANSparkMax m_mastMotor = new CANSparkMax((Constants.CanIds.kMastMotor),
      CANSparkMaxLowLevel.MotorType.kBrushless);

  public enum ManipulatorMode{
    AUTO, DISABLED, MANUAL
  }
  /** Creates a new ManipulatorSub. */
  public ManipulatorSub() {
    m_mastMotor.setInverted(true);
  }

  public void setManipulatorState(ManipulatorMode mode, double mastPower){
    //TODO others arent implented yet
    assert mode == ManipulatorMode.MANUAL;
    m_mastPower = mastPower; 
  }

  @Override
  public void periodic() {
    updateManipulatorStateMachine();
    updateSmartDashboard();
    // This method will be called once per scheduler run
  }
  
  //TODO make this private when moved into state machine
  public void rotateArm(double armPower) {
    m_armMotor.set(armPower);
  }

  private void moveMast(double mastPower) {
    m_mastMotor.set(mastPower);
  }

  private double getMastEncoder() {
    return m_mastMotor.getEncoder().getPosition();
  }

  private void updateManipulatorStateMachine(){
    if (getMastEncoder() >= kMastEncoderMax && m_mastPower > 0){
      moveMast(0); 
    }
    else if (getMastEncoder() <= kMastEncoderMin && m_mastPower < 0){
      moveMast(0);
    }
    else {
      moveMast (m_mastPower);
    }
  }
  
  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Mast Encoder Number", getMastEncoder());
  }



}
