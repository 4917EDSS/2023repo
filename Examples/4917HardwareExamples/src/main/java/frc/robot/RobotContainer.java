// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunSparkMaxWithJoystickCmd;
import frc.robot.commands.RunTalonFxWithJoystickCmd;
import frc.robot.subsystems.LimitSwitchSub;
import frc.robot.subsystems.NavXSub;
import frc.robot.subsystems.SolenoidSub;
import frc.robot.subsystems.SparkMaxSub;
import frc.robot.subsystems.TalonFxSub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final LimitSwitchSub m_limitSwitchSub = new LimitSwitchSub();
  private final NavXSub m_navXSub = new NavXSub();
  private final SolenoidSub m_solenoidSub = new SolenoidSub();
  private final SparkMaxSub m_sparkMaxSub = new SparkMaxSub();
  private final TalonFxSub m_talonFxSub = new TalonFxSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    m_driverController.cross().onTrue(new PrintCommand("Cross pressed"));

    // Instead of creating a command for this simple situation, define the command here
    m_driverController.square().whileTrue(
      new StartEndCommand(
        () -> m_solenoidSub.setSolenoid(true),   // Call on command start
        () -> m_solenoidSub.setSolenoid(false),  // Call on command end
        m_solenoidSub));                               // Required subsystem

    // Set the command to run when no other commands are using this subsystem
    m_sparkMaxSub.setDefaultCommand(new RunSparkMaxWithJoystickCmd(m_driverController, m_sparkMaxSub));
    m_talonFxSub.setDefaultCommand(new RunTalonFxWithJoystickCmd(m_driverController, m_talonFxSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No auto for this project");
  }

  // Called by robotPeriodic so we can display things to the dashboard
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", m_limitSwitchSub.isSwitchPressed());
    SmartDashboard.putNumber("NavX Yaw", m_navXSub.getAngle());
  }
}
