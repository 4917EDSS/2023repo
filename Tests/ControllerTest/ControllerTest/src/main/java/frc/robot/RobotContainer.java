// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.JoystickPrintCmd;
import frc.robot.subsystems.JoystickSub;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final JoystickSub m_joystickSub = new JoystickSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = 
    new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    m_joystickSub.setDefaultCommand(new JoystickPrintCmd(m_joystickSub, m_driverController));
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

    m_driverController.square().onTrue(new PrintCommand("Square Pressed"));
    m_driverController.circle().onTrue(new PrintCommand("Circle Pressed"));
    m_driverController.triangle().onTrue(new PrintCommand("Triangle Pressed"));
    m_driverController.cross().onTrue(new PrintCommand("Cross Pressed"));

    m_driverController.povUp().onTrue(new PrintCommand("POV Up Pressed"));
    m_driverController.povRight().onTrue(new PrintCommand("POV Right Pressed"));
    m_driverController.povDown().onTrue(new PrintCommand("POV Down Pressed"));
    m_driverController.povLeft().onTrue(new PrintCommand("POV Left Pressed"));
    m_driverController.povUpLeft().onTrue(new PrintCommand("POV Top Left Pressed"));
    m_driverController.povUpRight().onTrue(new PrintCommand("POV Top Right Pressed"));
    m_driverController.povDownRight().onTrue(new PrintCommand("POV Bottom Right Pressed"));
    m_driverController.povDownLeft().onTrue(new PrintCommand("POV Bottom Left Pressed"));

    m_driverController.share().onTrue(new PrintCommand("Share Pressed"));
    m_driverController.options().onTrue(new PrintCommand("Options Pressed"));
    m_driverController.touchpad().onTrue(new PrintCommand("Touchpad Pressed"));

    m_driverController.L1().onTrue(new PrintCommand("Left Bumper Pressed"));
    m_driverController.L2().onTrue(new PrintCommand("Left Trigger Pressed"));
    m_driverController.R1().onTrue(new PrintCommand("Right Bumper Pressed"));
    m_driverController.R2().onTrue(new PrintCommand("Right Trigger Pressed"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   **/
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No auto coded yet");
  }
}
