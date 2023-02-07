// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSub extends SubsystemBase {
  // CONSTANTS //
  private static final double kMastPositionMax = 60.0; // In encoder ticks (striaght up is 30)
  private static final double kMastPositionMin = 0.0; // In endcoder ticks
  private static final double kMastVelocityMax = 1.0; // TODO set to reasonable value
  private static final double kMastVelocityMin = -1.0; // TODO set to reasonable value

  private static final double kArmAngleMin = -80.0; // In encoder ticks
  private static final double kArmAngleMax = 80.0; // In encoder ticks
  private static final double kArmVelocityMax = 1.0; // TODO set to reasonable value
  private static final double kArmVelocityMin = -1.0; // TODO set to reasonable value

  private final double kArmPower = 0.6;
  private final double kMastPower = 0.3;

  // SUBSYTEM HARDWARE AND CONTROL OBJECTS //
  private final CANSparkMax m_armMotor =
      new CANSparkMax(Constants.CanIds.kArmMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CANSparkMax m_mastMotor =
      new CANSparkMax((Constants.CanIds.kMastMotor), CANSparkMaxLowLevel.MotorType.kBrushless);

  private final PIDController m_mastPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController m_armPID = new PIDController(0.1, 0.0, 0.0);

  // STATE VARIABLES //
  public enum OperationMode {
    AUTO, DISABLED, MANUAL
  }

  public enum OperationState {
    IDLE, HOLDING, MOVING, INTERRUPTED
  }

  private OperationMode m_mastCurrentMode = OperationMode.DISABLED;
  private OperationState m_mastNewState = OperationState.IDLE;
  private OperationMode m_mastNewMode = OperationMode.DISABLED;
  private double m_mastCurrentPosition = 0.0;
  private double m_mastCurrentVelocity = 0.0;
  private double m_mastTargetPosition = 0.0;
  private double m_mastNewTargetPosition = 0.0;
  private double m_mastTargetVelocity = 0.0;
  private double m_mastTargetPower = 0.0;
  private double m_mastNewTargetPower = 0.0;
  private boolean m_mastNewStateParameters = false;

  private double m_armCurrentAngle = 0.0;
  private double m_armCurrentVelocity = 0.0;
  private OperationMode m_armCurrentMode = OperationMode.DISABLED;

  /** Creates a new ManipulatorSub. */
  public ManipulatorSub() {
    zeroManipulator();

    m_mastMotor.getEncoder().setPosition(0.0); // Reset the encoders on startup
    m_armMotor.getEncoder().setPosition(0.0);
    m_mastMotor.setInverted(true);

    SmartDashboard.putNumber("Mast kP", 0.1);
    SmartDashboard.putNumber("Mast kD", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateManipulatorStateMachine();
    updateSmartDashboard();
  }

  /** Use this method to reset all of the hardware and states to safe starting values */
  public void init() {
    //TODO: Add resets here.  Call from constructor.
  }

  /** This method puts the subsystem in a safe state when all commands are interrupted */
  public void interrupt() {
    //TODO: Cleanly stop any state machine driven motion
  }

  public void setManipulatorState(OperationMode mode, double mastPower) {
    // TODO others arent implented yet
    assert mode == OperationMode.MANUAL;
    m_mastTargetPower = mastPower;
  }

  private void zeroManipulator() {
    m_armMotor.getEncoder().setPosition(0);
    m_mastMotor.getEncoder().setPosition(0);
  }

  public void resetEncoders() {
    m_mastMotor.getEncoder().setPosition(0.0);
    m_armMotor.getEncoder().setPosition(0.0);
  }

  private void updateManipulatorStateMachine() {
    //IF MODE MANUAL
    if(getMastPosition() >= kMastPositionMax && m_mastTargetPower > 0) {
      moveMast(0);
    } else if(getMastPosition() <= kMastPositionMin && m_mastTargetPower < 0) {
      moveMast(0);
    } else {
      moveMast(m_mastTargetPower);
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


  public void setMastPosition(OperationMode mode, double targetPower, double targetPosition) {
    // Only do something if one of the parameters has changed
    if((mode == m_mastCurrentMode) && (targetPower == m_mastTargetPower) && (targetPosition == m_mastTargetPosition)) {
      return;
    }

    // validate input parameters
    if(Math.abs(targetPower) > 1.0) {
      return;
    }

    if((mode != OperationMode.MANUAL) && ((targetPosition < kMastPositionMin) || (targetPosition > kMastPositionMax))) {
      return;
    }

    // If an old input is pedning, drop it
    m_mastNewStateParameters = false;

    switch(mode) {
      case DISABLED:
        // Turn motors off
        m_mastNewState = OperationState.IDLE;
        m_mastNewMode = mode;
        m_mastNewTargetPower = 0.0;
        m_mastNewTargetPosition = targetPosition;
        m_mastNewStateParameters = true; // Only set this to true after all the other parameters have been set
        break;
      case AUTO:
        // Go to new target position
        m_mastNewState = OperationState.MOVING;
        m_mastNewMode = mode;
        m_mastNewTargetPower = Math.abs(targetPower);
        m_mastNewTargetPosition = targetPosition;
        m_mastNewStateParameters = true; // Only set this to true after all the other parameters have been set
        break;
      case MANUAL:
        break;
    }

  }

  public boolean isMastWithinLimits() {
    boolean withinPositionLimits = false;
    boolean withinVelocityLimits = false;

    if((getMastPosition() > kMastPositionMin) && (getMastPosition() < kMastPositionMax)) {
      withinPositionLimits = true;
    }
    if((getMastVelocity() > kMastVelocityMin) && (getMastVelocity() < kMastVelocityMax)) {
      withinVelocityLimits = true;
    }

    return (withinPositionLimits && withinVelocityLimits);
  }

  public void moveMast(double mastPower) {
    m_mastMotor.set(mastPower);
  }

  public double getMastPosition() {
    return m_mastMotor.getEncoder().getPosition();
  }

  public double getMastVelocity() {
    return m_mastMotor.getEncoder().getVelocity();
  }

  public void setMastPosition(double encoderTicks) { // Set tick position of mast. 0 - Full back, 30 - Straight up, 60
    // full forwards
    double currentPos = getMastPosition();
    double targetPos = encoderTicks;
    double power = MathUtil.clamp(m_mastPID.calculate(currentPos, targetPos), -kMastPower, kMastPower); // Calculate how much power to use to get to target position

    moveMast(power);
  }

  public void setMastMode(OperationMode mode, double encoderTicks) {
    System.out.println("mode " + mode + " encoder ticks " + encoderTicks);
    if(mode == OperationMode.MANUAL) {
      m_mastTargetPosition = encoderTicks;
    }

  }
  // ------------------------- ARM -----------------------//

  // TODO make this private when moved into state machine

  public boolean isArmWithinLimits() {
    boolean withinAngleLimits = false;
    boolean withinVelocityLimits = false;

    if((getArmAngle() > kArmAngleMin) && (getArmAngle() < kArmAngleMax)) {
      withinAngleLimits = true;
    }
    if((getArmVelocity() > kArmVelocityMin) && (getArmVelocity() < kArmVelocityMax)) {
      withinVelocityLimits = true;
    }

    return (withinAngleLimits && withinVelocityLimits);
  }

  public void rotateArm(double armPower) {
    m_armMotor.set(armPower);
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
    double power = MathUtil.clamp(m_armPID.calculate(currentAngle, targetAngle), -kArmPower, kArmPower);

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
    double armP = 0.1;
    double armI = 0.0;
    double armD = 0.0;

    double mastP = 0.1;
    double mastI = 0.0;
    double mastD = 0.0;

    SmartDashboard.putNumber("Mast Encoder Number", getMastPosition());
    SmartDashboard.putNumber("Arm Encoder Number", getArmAngle());

    armP = SmartDashboard.getNumber("Arm kP", 0.1); // Get then set
    armI = SmartDashboard.getNumber("Arm kI", 0.0);
    armD = SmartDashboard.getNumber("Arm kD", 0.0);

    mastP = SmartDashboard.getNumber("Mast kP", 0.1);
    mastI = SmartDashboard.getNumber("Mast kI", 0.0);
    mastD = SmartDashboard.getNumber("Mast kD", 0.0);

    SmartDashboard.putNumber("Arm kP", armP);
    SmartDashboard.putNumber("Arm kI", armI);
    SmartDashboard.putNumber("Arm kD", armD);
    SmartDashboard.putNumber("Mast kP", mastP);
    SmartDashboard.putNumber("Mast kI", mastI);
    SmartDashboard.putNumber("Mast kD", mastD);

    m_armPID.setP(armP);
    m_armPID.setI(armI);
    m_armPID.setD(armD);
    m_mastPID.setP(mastP);
    m_mastPID.setI(mastI);
    m_mastPID.setD(mastD);

  }
}
