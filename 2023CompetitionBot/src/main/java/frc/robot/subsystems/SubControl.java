package frc.robot.subsystems;

/** Control variables for mechanism state machines */
public class SubControl {
  /**
   * Mode that the mechanism can operate in - DISABLED = Not running - AUTO = Automatically move to position - MANUAL =
   * Manual control, usually with joysticks
   */
  public enum Mode {
    DISABLED, AUTO, MANUAL
  }

  /**
   * State that the state machine is currently in - IDLE = Inactive - MOVING = Going to the target position - HOLDING =
   * Actively keeping mechanism at target position - INTERRUPTED = Something is preventing mechanism from moving to
   * position
   */
  public enum State {
    IDLE, MOVING, HOLDING, INTERRUPTED
  }

  public Mode mode = Mode.DISABLED;
  public State state = State.IDLE;
  public double targetPosition = 0.0;
  public double targetPower = 0.0;
}
