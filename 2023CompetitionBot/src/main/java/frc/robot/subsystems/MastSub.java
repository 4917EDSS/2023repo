// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubControl.State;

public class MastSub extends SubsystemBase {
  // CONSTANTS ////////////////////////////////////////////////////////////////
  private static final double kPositionMin = 0.0; // In endcoder ticks
  private static final double kPositionMax = 308.0; // In encoder ticks (straight up is 30)
  private static final double kManualModePowerDeadband = 0.05; // If manual power is less than this, assume power is 0
  private static final double kMaxPosDifference = 0.1; // Maximum difference between the target and current pos for the state to finish  <---- Must be tuned
  private static final double kMaxPowerStop = 0.1; // max amount of power for the state to finish                                        <--- Must be tuned
  private static final double kMastMinSafeZone = 110;
  private static final double kMastMaxSafeZone = 130;
  //TODO: Tune the two constants above


  // STATE VARIABLES //////////////////////////////////////////////////////////
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private SubControl m_newControl = new SubControl(); // New state to copy to current state when newStateParameters is true
  private boolean m_newControlParameters = false; // Set to true when ready to switch to new state
  private double m_lastPower = 0;
  private double m_blockedPosition;
  private ArmSub m_armSub;
  private IntakeSub m_intakeSub;

  // HARDWARE AND CONTROL OBJECTS /////////////////////////////////////////////
  private final CANSparkMax m_motor =
      new CANSparkMax((Constants.CanIds.kMastMotor), CANSparkMaxLowLevel.MotorType.kBrushless);

  private final DigitalInput m_backMast = new DigitalInput(Constants.DioIds.kMastBack);
  private final DigitalInput m_frontMast = new DigitalInput(Constants.DioIds.kMastFront);

  private double m_p = 0.1;
  private double m_i = 0.0;
  private double m_d = 0.0;
  private final PIDController m_pid = new PIDController(m_p, m_i, m_d);

  // SUBSYSTEM METHODS ////////////////////////////////////////////////////////
  /** Creates a new MastSub. */
  public MastSub() {
    m_motor.setInverted(false);

    SmartDashboard.putNumber("Mast kP", m_p);
    SmartDashboard.putNumber("Mast kI", m_i);
    SmartDashboard.putNumber("Mast kD", m_i);

    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateStateMachine();
    updateSmartDashboard();
  }

  /**
   * Use this method to reset all of the hardware and states to safe starting values
   */
  public void init() {
    zeroEncoder();
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * This method puts the subsystem in a safe state when all commands are interrupted
   */
  public void interrupt() {
    setPosition(SubControl.Mode.DISABLED, 0.0, 0.0);
  }

  /**
   * Blindly sets the mechanism power (-1.0 to 1.0). Use setPosition for smart operation
   */
  public void move(double power) {
    m_motor.set(power);
  }

  /** Sets the current position as the starting position - use wisely */
  public void zeroEncoder() {
    m_motor.getEncoder().setPosition(0);
  }

  /** Returns the position of the mechanism in encoder ticks */
  public double getPosition() {
    return m_motor.getEncoder().getPosition();
  }

  public boolean isSafeZone() {
    if((getPosition() > kMastMinSafeZone) && (getPosition() < kMastMaxSafeZone)) {
      return true;
    }
    return false;
  }

  public void setArmSub(ArmSub armSub) {
    m_armSub = armSub; 
  }

  public void setIntakeSub(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
  }

  /** Returns the velocity of the mechanism in ticks per second */
  public double getVelocity() {
    return m_motor.getEncoder().getVelocity();
  }

  public boolean isBlocked(double currentPosition, double targetPosition) {
    //TODO implement later
    if((currentPosition > targetPosition && isMastAtBack())) {
      return true;
    } 
    if((currentPosition < targetPosition && isMastAtFront())) {
      return true;
    } 
    return false;
  }

  public boolean isMastAtBack() {
    return m_backMast.get();
  }

  public boolean isMastAtFront() {
    return !m_frontMast.get();
  }

  /**
   * Move the mechanism to the desired position using the state machine - In mode DISABLED, the mechanism is disabled -
   * In mode AUTO, the mechanism smoothly goes to the specified position - In mode MANUAL, the mechanism blindly moves
   * in the specified direction with the specified power
   */
  public void setPosition(SubControl.Mode mode, double targetPower, double targetPosition) {
    // Only do something if one of the parameters has changed
    if((mode == m_currentControl.mode) && (targetPower == m_currentControl.targetPower)
        && (targetPosition == m_currentControl.targetPosition)) {
      return;
    }

    // Validate input parameters
    if(Math.abs(targetPower) > 1.0) {
      return; // Power out of range
    }

    if((mode != SubControl.Mode.MANUAL) && ((targetPosition < kPositionMin) || (targetPosition > kPositionMax))) {
      return; // Position is beyond allowable range
    }

    // If an old input is pending, drop it
    m_newControlParameters = false;

    // Set the new state machine parameters based on the specified control mode
    switch(mode) {
      case DISABLED:
        // Turn motors off
        m_newControl.state = SubControl.State.IDLE;
        m_newControl.mode = mode;
        m_newControl.targetPower = 0.0;
        m_newControl.targetPosition = 0.0;
        m_newControlParameters = true; // Only set this to true after all the other parameters have been set

        break;

      case AUTO:
        // Go to new target position
        m_newControl.state = SubControl.State.MOVING;
        m_newControl.mode = mode;
        m_newControl.targetPower = Math.abs(targetPower);
        m_newControl.targetPosition = targetPosition;
        m_newControlParameters = true; // Only set this to true after all the other parameters have been set
        break;

      case MANUAL:
        if(Math.abs(targetPower) < kManualModePowerDeadband) {
          // Power is 0 or close to 0 so hold position
          if(m_currentControl.state != SubControl.State.HOLDING) {
            // We're not currently holding so set that up
            m_newControl.state = SubControl.State.HOLDING;
            m_newControl.mode = mode;
            m_newControl.targetPower = 0.0;
            m_newControl.targetPosition = getPosition();
            m_newControlParameters = true;
          }
        } else {
          // Move in the specified direction with the specified power
          m_newControl.state = SubControl.State.MOVING;
          m_newControl.mode = mode;
          m_newControl.targetPower = Math.abs(targetPower);
          // Set the target position to be as far is the mechanism can go in the specified
          // direction
          if(targetPower > 0) {
            m_newControl.targetPosition = kPositionMax;
          } else {
            m_newControl.targetPosition = kPositionMin;
          }
          m_newControlParameters = true;
        }
        break;
    }
  }

  /** Run the mechanism state machine */
  private void updateStateMachine() {
    if(isMastAtBack()) {
      zeroEncoder();
    }
    double newPower = 0.0;
    double currentPosition = getPosition();

    // Check if there are new control parameters to set
    if(m_newControlParameters) {
      m_currentControl.state = m_newControl.state;
      m_currentControl.mode = m_newControl.mode;
      m_currentControl.targetPower = m_newControl.targetPower;
      m_currentControl.targetPosition = m_newControl.targetPosition;
      m_newControlParameters = false;
    }

    // Determine what power the mechanism should use based on the current state
    switch(m_currentControl.state) {
      case IDLE:
        // If the state machine is idle, don't supply any power to the mechanism
        newPower = 0.0;
        break;

      case MOVING:
        // If the mechanism is moving, check if it has arrived at it's target.
        if(isBlocked(currentPosition, m_currentControl.targetPosition)) {
          m_blockedPosition = currentPosition;
          m_currentControl.state = State.INTERRUPTED;
        } else if(isFinished()) {
          m_currentControl.state = State.HOLDING;
        } else {
          newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
        }

        // If not, check if it's blocked
        // If not, the set then calculate the move power
        // TODO: Add missing logic (see 2019 Elevator state machine)
        break;

      case HOLDING:
        // If the mechanism is at it's target location, apply power to hold it there if
        // necessary
        // TODO: Check if we can use the calcMovePower function since the PID could take
        // care of both cases
        newPower = calcHoldPower(currentPosition, m_currentControl.targetPosition);
        break;

      case INTERRUPTED:
        // If the mechanism is no longer blocked, transition to MOVING
        if(!isBlocked(currentPosition, m_currentControl.targetPosition)) {
          m_currentControl.state = State.MOVING;
          // Otherwise, hold this position
        } else {
          newPower = calcHoldPower(currentPosition, m_blockedPosition);
        }
        break;

      default:
        m_currentControl.state = State.HOLDING;
        break;
    }

    if(newPower != m_lastPower) {
      move(newPower);
      m_lastPower = newPower;
    }
  }


  /** Calculate the amount of power should use to get to the target position */
  private double calcMovePower(double currentPosition, double newPosition, double targetPower) {
    return MathUtil.clamp(m_pid.calculate(currentPosition, newPosition), -targetPower, targetPower);
  }


  private double calcHoldPower(double currentPosition, double targetPosition) {
    return 0;
  }

  /** Display/get subsystem information to/from the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Mast Encoder", getPosition());

    double p = SmartDashboard.getNumber("Mast kP", m_p);
    double i = SmartDashboard.getNumber("Mast kI", m_i);
    double d = SmartDashboard.getNumber("Mast kD", m_d);

    SmartDashboard.putNumber("Mast kP", p);
    SmartDashboard.putNumber("Mast kI", i);
    SmartDashboard.putNumber("Mast kD", d);
    SmartDashboard.putBoolean("Mast back", isMastAtBack());
    SmartDashboard.putBoolean("Mast front", isMastAtFront());

    m_pid.setP(p);
    m_pid.setI(i);
    m_pid.setD(d);
  }

  public boolean isFinished() {
    if(Math.abs(getPosition() - m_currentControl.targetPosition) > kMaxPosDifference) {
      return false;
    }
    if(Math.abs(getVelocity()) > kMaxPowerStop) {
      return false;
    }
    if(m_newControlParameters) {
      return false;
    }
    return true;
  }

}
