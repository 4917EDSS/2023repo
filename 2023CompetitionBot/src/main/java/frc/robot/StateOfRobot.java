package frc.robot;

// import frc.robot.subsystems.ManipulatorsPositions;

public class StateOfRobot {
  public static boolean m_coneMode = false;//if false assume cubemode, if true assume conemdoe
  // TODO: we are simplifying for now, just going to set positions directly.
  // public static IntakePositions m_currentTargetLocation = IntakePositions.START;
  public static boolean m_operatorJoystickforIntake = false;

  public static boolean isCubeMode() {
    return (m_coneMode == false);
  }

  public static boolean isConeMode() {
    return (m_coneMode == true);
  }
}
