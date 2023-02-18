// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MastWithJoystickCmd;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MastSub;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MastSub m_mastSub = new MastSub();
  private final ArmSub m_armSub = new ArmSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  
  // This is used to keep track of which subsystem the joystick input should get routed to
  private enum JoystickMode {
    MAST, ARM
  }
  JoystickMode m_currentJoystickMode = JoystickMode.MAST;

  // This is used to keep track of where we want to pick-up or deliver objects so we can call the appropriate command
  private enum TargetLocation {
    PICKUP_SINGLE_SUBSTATION, PICKUP_DOUBLE_SUBSTATION, PICKUP_GROUND, RETRACT, DELIVER_HIGH, DELIVER_MID, DELIVER_LOW
  }
  TargetLocation m_currentTargetLocation = TargetLocation.RETRACT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

   // m_mastSub.setDefaultCommand(m_mastJoystickSelectCommand);
   // m_armSub.setDefaultCommand(m_armJoystickSelectCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.options().onTrue(new PrintCommand("Button Pressed!"));

    // These buttons set the subsystem that the left Y joystick controls
    m_driverController.touchpad().onTrue(new InstantCommand(() -> m_currentJoystickMode = JoystickMode.MAST));
    m_driverController.PS().onTrue(new InstantCommand(() -> m_currentJoystickMode = JoystickMode.ARM));

    // These buttons set the pick-up or delivery location m_currentTargetLocation = targetLocation setTargetLocation(TargetLocation.PICKUP_DOUBLE_SUBSTATION)));
    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.PICKUP_SINGLE_SUBSTATION));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.PICKUP_GROUND));
    m_driverController.square().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.RETRACT));
    m_driverController.triangle().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.DELIVER_HIGH));
    m_driverController.circle().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.DELIVER_MID));
    m_driverController.cross().onTrue(new InstantCommand(() -> m_currentTargetLocation = TargetLocation.DELIVER_LOW));

    // This button changes behaviour based on the target location
    m_driverController.R2().onTrue(m_targetLocationSelectCommand);
  }

  private final Command m_mastJoystickSelectCommand = new SelectCommand(
      Map.ofEntries(
          Map.entry(JoystickMode.MAST, new MastWithJoystickCmd(true, m_driverController, m_mastSub)),
          Map.entry(JoystickMode.ARM, new MastWithJoystickCmd(false, m_driverController, m_mastSub))),
      this::getJoystickMode);

  private final Command m_armJoystickSelectCommand = new SelectCommand(
      Map.ofEntries(
          Map.entry(JoystickMode.MAST, new MastWithJoystickCmd(true, m_driverController, m_mastSub)),
          Map.entry(JoystickMode.ARM, new MastWithJoystickCmd(false, m_driverController, m_mastSub))),
      this::getJoystickMode);

  private JoystickMode getJoystickMode() {
    return m_currentJoystickMode;
  }

  private final Command m_targetLocationSelectCommand = new SelectCommand(
      // Maps selector values to commands
      Map.ofEntries(
          Map.entry(TargetLocation.PICKUP_SINGLE_SUBSTATION, new PrintCommand("Pick-up at single station")),
          Map.entry(TargetLocation.PICKUP_DOUBLE_SUBSTATION, new PrintCommand("Pick-up at double station")),
          Map.entry(TargetLocation.PICKUP_GROUND, new PrintCommand("Pick-up off of ground")),
          Map.entry(TargetLocation.RETRACT, new PrintCommand("Retract mast and arm")),
          Map.entry(TargetLocation.DELIVER_HIGH, new PrintCommand("Deliver high")),
          Map.entry(TargetLocation.DELIVER_MID, new PrintCommand("Deliver mid")),
          Map.entry(TargetLocation.DELIVER_LOW, new PrintCommand("Deliver low"))),
      this::getTargetLocation);

  private TargetLocation getTargetLocation() {
    return m_currentTargetLocation;
  }

  private void setTargetLocation(TargetLocation targetLocation) {
    m_currentTargetLocation = targetLocation;
    System.out.println(targetLocation.name());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void updateDashboard() {
    SmartDashboard.putString("JoystickMode", m_currentJoystickMode.name());
    SmartDashboard.putString("TargetPos", m_currentTargetLocation.name());
  }
}
