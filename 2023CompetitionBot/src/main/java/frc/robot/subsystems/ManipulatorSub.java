// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ManipulatorSub extends SubsystemBase {

  // DEFAULT VARIABLES //

  private static final double kMastPositionMax = 60.0; // In encoder ticks
  private static final double kMastPositionMin = 0.0; // In endcoder ticks
  private static final double kMastVelocityMax = 1.0; // TODO fix me
  private static final double kMastVelocityMin = -1.0; // TODO fix me

  private static final double kArmAngleMin = -80.0; // In encoder ticks
  private static final double kArmAngleMax = 80.0; // In encoder ticks
  private static final double kArmVelocityMax = 1.0; // TODO fix me
  private static final double kArmVelocityMin = -1.0; // TODO fix me

  private final double kArmPower = 0.6;
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
  private boolean m_isAlive = true;

  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.CanIds.kArmMotor,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CANSparkMax m_mastMotor = new CANSparkMax((Constants.CanIds.kMastMotor),
      CANSparkMaxLowLevel.MotorType.kBrushless);

  public enum ManipulatorMode {
    AUTO, DISABLED, MANUAL
  }

  // STATE VARIABLES //

  private double m_mastCurrentPosition = 0.0;
  private double m_mastCurrentVelocity = 0.0;
  private ManipulatorMode m_mastCurrentMode = ManipulatorMode.DISABLED;
  private double m_mastTargettPosition = 0.0;
  private double m_mastTargetVelocity = 0.0;

  private double m_armCurrentAngle = 0.0;
  private double m_armCurrentVelocity = 0.0;
  private ManipulatorMode m_armCurrentMode = ManipulatorMode.DISABLED;

  /** Creates a new ManipulatorSub. */
  public ManipulatorSub() {
    zeroManipulator();

    m_mastMotor.getEncoder().setPosition(0.0); // Reset the encoders on startup
    m_armMotor.getEncoder().setPosition(0.0);
    m_mastMotor.setInverted(true);

    SmartDashboard.putNumber("Mast kP", 0.1);
    SmartDashboard.putNumber("Mast kD", 0.0);
  }

  public void setManipulatorState(ManipulatorMode mode, double mastPower) {
    // TODO others arent implented yet
    assert mode == ManipulatorMode.MANUAL;
    m_mastPower = mastPower;
  }

  private void zeroManipulator() {
    m_armMotor.getEncoder().setPosition(0);
    m_mastMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    updateManipulatorStateMachine();
    updateSmartDashboard();
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    m_mastMotor.getEncoder().setPosition(0.0); // Reset the encoders on startup
    m_armMotor.getEncoder().setPosition(0.0);
  }

  private void updateManipulatorStateMachine() {
    //IF MODE MANUAL
    if (getMastPosition() >= kMastPositionMax && m_mastPower > 0) {
      moveMast(0);
    } else if (getMastPosition() <= kMastPositionMin && m_mastPower < 0) {
      moveMast(0);
    } else {
      moveMast(m_mastPower);
    }

    //IF MODE AUTO -----
    // Is there a new target?
    //   If yes, then update all state machine variables with new ones
    //
    // Switch statement for mechanism state (IDLE, HOLDING, MOVING, INTERRUPTED)
    // case IDLE
    //  leave everything off
    // case HOLDING
    //  setHoldPower
    // case MOVING
    //  Am I interrupted?
    //    If yes, go to INTERRUPTED state
    //  Am I at target position?
    //    If yes, go to HOLDING state
    //  setMastPosition
    // case INTERRUPTED
    //  Am I still interrupted?
    //    If not, go to MOVING state
    //  setHoldPower
    
  }

  // ------------------------ GRIPPER -------------------//

  // Set mast power to 'power'
  public void setGripperToPosition(double MastEncoderPosition, double armEncoderPosition) {
    // TODO later during the lifecycle
    // double x = getMastPosition();
    // double y = getArmPosition();
    // while (x != MastEncoderPosition)
    // while (y != getArmPosition) {
    // x = getMastPosition();
    // y = getArmPosition();

    // updateManipulatorStateMachine();
    // }

  }

  // ------------------------- MAST ----------------------//

  public boolean isMastWithinLimits() {
    boolean withinPositionLimits = false;
    boolean withinVelocityLimits = false;

    if ((getMastPosition() > kMastPositionMin) && (getMastPosition() < kMastPositionMax)) {
      withinPositionLimits = true;
    }
    if ((getMastVelocity() > kMastVelocityMin) && (getMastVelocity() < kMastVelocityMax)) {
      withinVelocityLimits = true;
    }

    return (withinPositionLimits && withinVelocityLimits);
  }

  public void moveMast(double mastPower) {
    if (m_isAlive){
      m_mastMotor.set(mastPower);
    }
  }

  public void kill(){
    m_isAlive = false;
  }

  public double getMastPosition() {
    return m_mastMotor.getEncoder().getPosition();
  }

  public double getMastVelocity() {
    return m_mastMotor.getEncoder().getVelocity();
  }

  public void setMastPosition(double encoderTicks) { // Set tick position of mast. 0 - Full back, 30 - Straight up, 60
    // full forwards
    double currentPos = getMastPosition();// / kMaxMastTicks * 2.0 - 1.0; // Convert from 0-1 to -1-1
    double targetPos = encoderTicks;// MathUtil.clamp(encoderTicks, 0.0, kMaxMastTicks) / kMaxMastTicks * 2.0 - 1.0;
    double power = MathUtil.clamp(kMastPID.calculate(currentPos, targetPos), -kMastPower, kMastPower);

    moveMast(power);
  }

  public void setMastMode(ManipulatorMode mode, double encoderTicks) {
    System.out.println("mode " + mode + " encoder ticks " + encoderTicks);
    if (mode == ManipulatorMode.MANUAL) {
      m_mastTargettPosition = encoderTicks;
    }

  }
  // ------------------------- ARM -----------------------//

  // TODO make this private when moved into state machine

  public boolean isArmWithinLimits() {
    boolean withinAngleLimits = false;
    boolean withinVelocityLimits = false;

    if ((getArmAngle() > kArmAngleMin) && (getArmAngle() < kArmAngleMax)) {
      withinAngleLimits = true;
    }
    if ((getArmVelocity() > kArmVelocityMin) && (getArmVelocity() < kArmVelocityMax)) {
      withinVelocityLimits = true;
    }

    return (withinAngleLimits && withinVelocityLimits);
  }

  public void rotateArm(double armPower) {
    if (m_isAlive){
    m_armMotor.set(armPower);
    }
  }
    

  public double getArmAngle() {
    return m_armMotor.getEncoder().getPosition();
  }

  public double getArmVelocity() {
    return m_armMotor.getEncoder().getVelocity();
  }

  public void setArmAngle(double angle) { // 160 degrees of rotation (-80 to 80)
    double currentAngle = getArmAngle();// / kArmAngleMax; // -80, 80 to -1, 1 range
    double targetAngle = angle;//MathUtil.clamp(angle, kArmAngleMin, kArmAngleMax) / kArmAngleMax;
    double power = MathUtil.clamp(kArmPID.calculate(currentAngle, targetAngle), -kArmPower, kArmPower);

    // Set arm power to 'power'
    rotateArm(power);
  }

  public boolean isWithinLimits() {
    return (isMastWithinLimits() && isArmWithinLimits());
  }

  public void updateCurrentState() {
    System.out.println("SetManupulatorPositionCmd #Update current state");

    m_mastCurrentPosition = getMastPosition();
    m_mastCurrentVelocity = getMastVelocity();

    m_armCurrentAngle = getArmAngle();
    m_armCurrentVelocity = getArmVelocity();
  }

  // -------------------------- DASHBOARD ----------------------//

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Mast Encoder Number", getMastPosition());
    SmartDashboard.putNumber("Arm Encoder Number", getArmAngle());

    kArmP = SmartDashboard.getNumber("Arm kP", 0.1); // Get then set
    kArmI = SmartDashboard.getNumber("Arm kI", 0.0);
    kArmD = SmartDashboard.getNumber("Arm kD", 0.0);

    kMastP = SmartDashboard.getNumber("Mast kP", 0.1);
    kMastI = SmartDashboard.getNumber("Mast kI", 0.0);
    kMastD = SmartDashboard.getNumber("Mast kD", 0.0);

    SmartDashboard.putNumber("Arm kP", kArmP);
    SmartDashboard.putNumber("Arm kI", kArmI);
    SmartDashboard.putNumber("Arm kD", kArmD);
    SmartDashboard.putNumber("Mast kP", kMastP);
    SmartDashboard.putNumber("Mast kI", kMastI);
    SmartDashboard.putNumber("Mast kD", kMastD);

    kArmPID.setP(kArmP);
    kArmPID.setI(kArmI);
    kArmPID.setD(kArmD);
    kMastPID.setP(kMastP);
    kMastPID.setI(kMastI);
    kMastPID.setD(kMastD);

  }
}
