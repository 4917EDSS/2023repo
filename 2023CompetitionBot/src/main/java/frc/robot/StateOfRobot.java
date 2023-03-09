package frc.robot;

// import frc.robot.subsystems.ManipulatorsPositions;

public class StateOfRobot {
  public static boolean m_coneMode = false; // If false assume cube mode, if true assume cone mode
  public static boolean m_operatorJoystickforIntake = false;

  public static boolean isCubeMode() {
    return (m_coneMode == false);
  }

  public static boolean isConeMode() {
    return (m_coneMode == true);
  }
}
