// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ManipulatorSub extends SubsystemBase {
  private static final double kMastEncoderMax = 60;
  private static final double kMastEncoderMin = 0;
  private static final double kArmEncoderMin = -80.0; // 0 is within the center of the mast
  private static final double kArmEncoderMax = 80.0; // Positive towards spark maxes, negative away from them

  private final double kArmPower = 0.3;
  private final double kMastPower = 0.3;
  private final double kMaxMastTicks = 0.0; // 0 - 60.0 30 is straight up
  
  private double kArmP = 0.1;
  private double kArmI = 0.0;
  private double kArmD = 0.0;

  private double kMastP = 0.1;
  private double kMastI = 0.0;
  private double kMastD = 0.0;

  private final PIDController kMastPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController kArmPID = new PIDController(0.1, 0.0, 0.0);

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
    m_mastMotor.getEncoder().setPosition(0.0); // Reset the encoders on startup
    m_armMotor.getEncoder().setPosition(0.0);
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
    return m_mastMotor.getEncoder().getPosition() * -1;
  }
  private double getArmEncoder() {
    return m_armMotor.getEncoder().getPosition();
  }

  public void setArmAngle(double angle) { //160 degrees of rotation (-80 to 80)
    double currentAngle = getArmEncoder() / kArmEncoderMax ; // -80, 80 to -1, 1 range
    double targetAngle = MathUtil.clamp(angle,kArmEncoderMin,kArmEncoderMax) / kArmEncoderMax;
    double power = MathUtil.clamp(kArmPID.calculate(currentAngle,targetAngle),-kArmPower,kArmPower);

    // Set arm power to 'power'
  }
  public void setMastPosition(double encoderTicks) { // Set tick position of mast. 0 - Full back, 30 - Straight up, 60 full forwards
    double currentPos = getMastEncoder()/kMaxMastTicks * 2.0 - 1.0; //Convert from 0-1 to -1-1
    double targetPos = MathUtil.clamp(encoderTicks,0.0,kMaxMastTicks)/kMaxMastTicks * 2.0 - 1.0;
    double power = MathUtil.clamp(kMastPID.calculate(currentPos,targetPos),-kMastPower,kMastPower);

    // Set mast power to 'power'
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
    SmartDashboard.putNumber("Arm Encoder Number", getArmEncoder());

    kArmP = SmartDashboard.getNumber("Arm kP",0.1); // Get then set
    kArmI = SmartDashboard.getNumber("Arm kI",0.0);
    kArmD = SmartDashboard.getNumber("Arm kD",0.0);

    kMastP = SmartDashboard.getNumber("Mast kP",0.1);
    kMastI = SmartDashboard.getNumber("Mast kI",0.0);
    kMastD = SmartDashboard.getNumber("Mast kD",0.0);

    SmartDashboard.putNumber("Arm kP",kArmP); 
    SmartDashboard.putNumber("Arm kI",kArmI);
    SmartDashboard.putNumber("Arm kD",kArmD);
    SmartDashboard.putNumber("Mast kP",kMastP);
    SmartDashboard.putNumber("Mast kI",kMastI);
    SmartDashboard.putNumber("Mast kD",kMastD);

    kArmPID.setP(kArmP);
    kArmPID.setI(kArmI);
    kArmPID.setD(kArmD);
    kMastPID.setP(kMastP);
    kMastPID.setI(kMastI);
    kMastPID.setD(kMastD);

  }
}
