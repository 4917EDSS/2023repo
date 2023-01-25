// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.ManipulatorSub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ManipulatorSub m_manipulatorSub = new ManipulatorSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final GripperSub m_gripperSub = new GripperSub();
  // TODO: Add vision subsystem when camera connected

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController = new CommandPS4Controller(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Set default command for subsystems
    m_drivetrainSub.setDefaultCommand(new DriveWithJoystickCmd(m_driverController, m_drivetrainSub));
    // m_manipulatorSub.setDefaultCommand(new
    // RotateArmWithJoystickCmd(m_driverController, m_manipulatorSub));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.povUp().whileTrue(new PrintCommand("Arrow up pressed!!!!!!!"));
    m_driverController.povDown().whileTrue(new PrintCommand("Arrow down pressed!!!!!!!"));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // This ties a button to a command
    // m_driverController = The controller who's button you want to use
    // square() = The button you want to use (others: L1, cross, etc.)
    // onTrue = When you want the command to trigger (others: whileTrue,
    // toggleOnFalse, etc.)
    // DriveForwardCmd = The command you want to trigger
    // (m_drivetrainSub) = The parameters that the command needs
    // m_driverController.square().whileTrue(new DriveForwardCmd(m_drivetrainSub));

    // Instead of creating a command for this simple situation, define the command
    // here
    /*
     * m_driverController.circle().whileTrue( // This doens't help
     * new StartEndCommand(
     * () -> m_drivetrainSub.tankDrive(-0.25, 0.25), // Call on command start
     * () -> m_drivetrainSub.tankDrive(0.0, 0.0), // Call on command end
     * m_drivetrainSub)); // Required subsystem
     */

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_operatorController.povUp().whileTrue(
        new StartEndCommand(
            () -> m_manipulatorSub.rotateArm(0.3), // Call on command start
            () -> m_manipulatorSub.rotateArm(0.0), // Call on command end
            m_manipulatorSub)); // Required subsystem
    m_operatorController.povDown().whileTrue(
        new StartEndCommand(
            () -> m_manipulatorSub.rotateArm(-0.3),
            () -> m_manipulatorSub.rotateArm(0.0),
            m_manipulatorSub));

    m_operatorController.povRight().whileTrue(
        new StartEndCommand(() -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.6),
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.0), m_manipulatorSub));
    m_operatorController.povLeft()
        .whileTrue(new StartEndCommand(
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, -0.6),
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.0), m_manipulatorSub));
    m_operatorController.triangle().onTrue(
        new StartEndCommand(() -> m_gripperSub.setValve(true), () -> m_gripperSub.setValve(true), m_gripperSub));
    m_operatorController.cross().onTrue(
        new StartEndCommand(() -> m_gripperSub.setValve(false), () -> m_gripperSub.setValve(false), m_gripperSub));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No auto yet");
  }
}
