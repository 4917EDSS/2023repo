// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.SubControl.Mode;
import frc.robot.subsystems.SubControl.State;

public class ArmSub extends SubsystemBase {
  // CONSTANTS ////////////////////////////////////////////////////////////////
  private static final double kPositionMin = -240000.0; // In encoder ticks
  private static final double kPositionMax = 255000.0; // In encoder ticks (straight up is 30)
  private static final double kManualModePowerDeadband = 0.05; // If manual power is less than this, assume power is 0
  private static final double kMaxPosDifference = 1000; // Maximum difference between the target and current pos for the state to finish   <---- Must be tuned
  private static final double kMaxSpeedStop = 1000; // Max amount of power for the state to finish <--- Must be tuned
  private static final double kMaxDangerZone = 70061;
  private static final double kMinDangerZone = -70348;

  public static final double kVertical = 7760;

  // STATE VARIABLES //////////////////////////////////////////////////////////
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private SubControl m_newControl = new SubControl(); // New state to copy to current state when newStateParameters is true
  private boolean m_newControlParameters = false; // Set to true when ready to switch to new state
  private double m_lastPower = -999;
  private double m_blockedPosition;
  private IntakeSub m_intakeSub;
  private LedSub m_ledSub;

  // HARDWARE AND CONTROL OBJECTS /////////////////////////////////////////////
  private final TalonFX m_motorR = new TalonFX(Constants.CanIds.kArmMotorR);
  private final TalonFX m_motorL = new TalonFX(Constants.CanIds.kArmMotorL);

  private final DigitalInput m_armLimit = new DigitalInput(Constants.DioIds.kArmLimitPort);

  private double m_p = 0.0001;
  private double m_i = 0.0;
  private double m_d = 0.0;
  private final PIDController m_pid = new PIDController(m_p, m_i, m_d);

  // SUBSYSTEM METHODS ////////////////////////////////////////////////////////

  /** Creates a new ArmSub. */
  public ArmSub(MastSub mastSub, IntakeSub intakeSub, LedSub ledSub) {
    m_motorL.setInverted(true);

    m_ledSub = ledSub;
    m_intakeSub = intakeSub;
    m_intakeSub.setArmSub(this);

    if(Constants.kEnhanceDashBoard == true) {
      SmartDashboard.putNumber("Arm kP", m_p);
      SmartDashboard.putNumber("Arm kI", m_i);
      SmartDashboard.putNumber("Arm kD", m_d);
    }

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
    m_motorR.setNeutralMode(NeutralMode.Brake);
    m_motorL.setNeutralMode(NeutralMode.Brake);

    setPosition(Mode.DISABLED, 0, 0);
  }

  public void initTest() {
    zeroEncoder();
    m_motorR.setNeutralMode(NeutralMode.Coast);
    m_motorL.setNeutralMode(NeutralMode.Coast);

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
    m_motorR.set(ControlMode.PercentOutput, power);
    m_motorL.set(ControlMode.PercentOutput, power);

  }

  public void setBrake(boolean brake) {
    if(brake) {
      m_motorR.setNeutralMode(NeutralMode.Brake);
      m_motorL.setNeutralMode(NeutralMode.Brake);

    } else {
      m_motorR.setNeutralMode(NeutralMode.Coast);
      m_motorL.setNeutralMode(NeutralMode.Coast);

    }
  }

  /** Sets the current position as the starting position - use wisely */
  public void zeroEncoder() {
    m_motorR.setSelectedSensorPosition(0);
    m_motorL.setSelectedSensorPosition(0);

  }

  public void setEncoderPosition(double pos) {
    m_motorR.setSelectedSensorPosition(pos);
    m_motorL.setSelectedSensorPosition(pos);

  }

  /** Returns the position of the mechanism in encoder ticks */
  public double getPosition() {
    return m_motorR.getSelectedSensorPosition();
  }

  /** Returns the velocity of the mechanism in ticks per second */
  public double getVelocity() {
    return m_motorR.getSelectedSensorVelocity();
  }

  public boolean isDangerZone() {
    if((getPosition() >= kMinDangerZone) && (getPosition() <= kMaxDangerZone)) {
      return true;
    }
    return false;
  }

  public boolean isArmAtLimit() {
    return !m_armLimit.get();
  }

  public boolean aboveDangerZone(double currentPos) {
    return (currentPos > kMaxDangerZone);
  }

  public boolean belowDangerZone(double currentPos) {
    return (currentPos < kMinDangerZone);
  }

  public double distToDanger(double currentPos) {
    double dist = 0;
    if(aboveDangerZone(currentPos)) {
      dist = currentPos - kMaxDangerZone;
    } else if(belowDangerZone(currentPos)) {
      dist = kMinDangerZone - currentPos;
    }

    return dist;
  }

  public boolean isBlocked(double currentPosition, double targetPosition) {

    if(!isDangerZone()) {
      m_ledSub.setZoneColour(LedZones.ARM_BLOCKED, LedColour.GREEN);
      return false;
    } else {
      if(!m_intakeSub.isSafeZone()) {
        m_ledSub.setZoneColour(LedZones.ARM_BLOCKED, LedColour.RED);
        return true;
      }
    }
    return false;

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
            m_newControl.targetPower = 0.30;
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
        break;

      case HOLDING:
        // If the mechanism is at it's target location, apply power to hold it there if necessary
        newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
        break;

      case INTERRUPTED:
        // If the mechanism is no longer blocked, transition to MOVING
        if(!isBlocked(currentPosition, m_currentControl.targetPosition)) {
          m_currentControl.state = State.MOVING;
          // Otherwise, hold this position
        } else {
          newPower = calcMovePower(currentPosition, m_blockedPosition, m_currentControl.targetPower);
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
    if(Math.abs(currentPosition - newPosition) < 20000) {
      if(targetPower > 0.4) {
        targetPower = 0.4;
      }
    } else if(Math.abs(currentPosition - newPosition) < 40000) {
      if(targetPower > 0.5) {
        targetPower = 0.5;
      }
    }
    return MathUtil.clamp(m_pid.calculate(currentPosition, newPosition), -targetPower, targetPower);


  }


  public boolean isFinished() {
    if(Math.abs(getPosition() - m_currentControl.targetPosition) > kMaxPosDifference) {
      return false;
    }
    if(Math.abs(getVelocity()) > kMaxSpeedStop) {
      return false;
    }
    if(m_newControlParameters) {
      return false;
    }
    return true;
  }

  /** Display/get subsystem information to/from the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Encoder", getPosition());
    SmartDashboard.putBoolean("Arm Limit Sensor", isArmAtLimit());

    if(Constants.kEnhanceDashBoard == true) {
      double p = SmartDashboard.getNumber("Arm kP", m_p);
      double i = SmartDashboard.getNumber("Arm kI", m_i);
      double d = SmartDashboard.getNumber("Arm kD", m_d);

      SmartDashboard.putNumber("Arm kP", p);
      SmartDashboard.putNumber("Arm kI", i);
      SmartDashboard.putNumber("Arm kD", d);

      m_pid.setP(p);
      m_pid.setI(i);
      m_pid.setD(d);
    }
  }
}
