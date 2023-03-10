// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateOfRobot;

public class IntakeSub extends SubsystemBase {
  private static final double kPositionMin = 0; // In encoder ticks
  private static final double kPositionMax = 60.0; // In encoder ticks (straight up is 30)
  private static final double kManualModePowerDeadband = 0.05; // If manual power is less than this, assume power is 0
  private static final double kIntakeMinSafeZone = 0.0;
  private static final double kIntakeMaxSafeZone = 18.0;
  public static final double kWristFlush = 27;
  public static final double kWristThrough = 12.595;
  public static final double kMaxPosDifference = 0.1;


  // STATE VARIABLES //////////////////////////////////////////////////////////
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private SubControl m_newControl = new SubControl(); // New state to copy to current state when newStateParameters is true
  private boolean m_newControlParameters; // Set to true when ready to switch to new state
  private double m_lastPower = 0;

  private final CANSparkMax m_intakeMotor =
      new CANSparkMax((Constants.CanIds.kIntakeMotor), CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rotateMotor =
      new CANSparkMax((Constants.CanIds.kRotateMotor), CANSparkMaxLowLevel.MotorType.kBrushless);

  // private final DigitalInput m_cubeSensor = new DigitalInput(Constants.DioIds.kCubeSensorPort);
  // private final DigitalInput m_coneSensor = new DigitalInput(Constants.DioIds.kConeSensorPort);
  private final DigitalInput m_intakeLimitSwitch = new DigitalInput(Constants.DioIds.kIntakeLimitPort);
  private final AnalogInput m_gamePieceSensor = new AnalogInput(Constants.AnalogInputIds.kGamePieceSensorPort);

  private double m_p = 0.1;
  private double m_i = 0.0;
  private double m_d = 0.0;
  private final PIDController m_pid = new PIDController(m_p, m_i, m_d);

  /** Creates a new intakeSub. */
  public IntakeSub() {

  }

  @Override
  public void periodic() {
    updateStateMachine();
    updateSmartDashboard();
  }

  /** Use this method to reset all of the hardware and states to safe starting values */
  public void init() {
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_rotateMotor.setIdleMode(IdleMode.kBrake);
  }

  public void initTest() {
    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_rotateMotor.setIdleMode(IdleMode.kCoast);
    zeroEncoderIntake();
    zeroEncoderRotate();
  }

  /** This method puts the subsystem in a safe state when all commands are interrupted */
  public void interrupt() {

  }

  // This is for the intake motors
  public void spinWheelsIntake(double power) {
    m_intakeMotor.set(power);
  }

  public void zeroEncoderIntake() {
    m_intakeMotor.getEncoder().setPosition(0);
  }

  /** Returns the velocity of the mechanism in ticks per second */
  public double getVelocityIntake() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public boolean isSafeZone() {
    if((getPositionRotate() < kIntakeMaxSafeZone) && (getPositionRotate() > kIntakeMinSafeZone)) {
      return true;
    }
    return false;
  }

  // This is for the rotate motors
  public void intakeRotate(double power) {
    m_rotateMotor.set(power);
  }

  public void zeroEncoderRotate() {
    m_rotateMotor.getEncoder().setPosition(0);
  }

  /** Returns the position of the mechanism in encoder ticks */
  public double getPositionRotate() {
    return m_rotateMotor.getEncoder().getPosition();
  }

  /** Returns the velocity of the mechanism in ticks per second */
  public double getVelocityRotate() {
    return m_rotateMotor.getEncoder().getVelocity();
  }

  public boolean isIntakeLoaded() {
    if((StateOfRobot.isConeMode() == true) && (m_gamePieceSensor.getValue() > 400)) {
      return true;
    }
    if((StateOfRobot.isCubeMode() == true) && (m_gamePieceSensor.getValue() > 400)) {
      return true;
    }
    return false;
  }

  public boolean isIntakeAtLimit() {
    return !m_intakeLimitSwitch.get();
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
            m_newControl.targetPosition = getPositionRotate();
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
    double currentPosition = getPositionRotate();
    if(isIntakeAtLimit()) {
      zeroEncoderRotate();
    }

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
        // If not, check if it's blocked
        // If not, the set then calculate the move power
        // TODO: Add missing logic (see 2019 Elevator state machine)
        newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
        break;

      case HOLDING:
        // If the mechanism is at it's target location, apply power to hold it there if necessary
        // TODO: Check if we can use the calcMovePower function since the PID could take care of both cases
        newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
        break;

      case INTERRUPTED:
        // If the mechanism is no longer blocked, transition to MOVING
        // Otherwise, hold this position
        // TODO: Add missing logic (see 2019 Elevator state machine)
        break;
    }

    if(newPower != m_lastPower) {
      intakeRotate(newPower);
      m_lastPower = newPower;
    }
  }

  /** Calculate the amount of power should use to get to the target position */
  private double calcMovePower(double currentPosition, double newPosition, double targetPower) {
    return MathUtil.clamp(m_pid.calculate(currentPosition, newPosition), -targetPower, targetPower);
  }

  // private double calcHoldPower(double currentPosition) {
  //   // TODO: Decide what is needed to hold the position
  //   // We probalby just want to use calcMovePower
  //   return 0.0;
  // }

  /** Display/get subsystem information to/from the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Rotate Intake Encoder", getPositionRotate());

    double p = SmartDashboard.getNumber("Intake kP", m_p);
    double i = SmartDashboard.getNumber("Intake kI", m_i);
    double d = SmartDashboard.getNumber("Intake kD", m_d);

    SmartDashboard.putNumber("Intake kP", p);
    SmartDashboard.putNumber("Intake kI", i);
    SmartDashboard.putNumber("Intake kD", d);
    SmartDashboard.putNumber("Arm Voltage", m_intakeMotor.getOutputCurrent());

    SmartDashboard.putNumber("Piece Sensor", m_gamePieceSensor.getValue());
    SmartDashboard.putBoolean("Intake Limit", isIntakeAtLimit());
    SmartDashboard.putBoolean("Intake Loaded", isIntakeLoaded());

    m_pid.setP(p);
    m_pid.setI(i);
    m_pid.setD(d);
  }

  public boolean isFinished() {
    if(Math.abs(getPositionRotate() - m_currentControl.targetPosition) > kMaxPosDifference) {
      return false;
    }
    if(m_newControlParameters) {
      return false;
    }
    return true;
  }

}
