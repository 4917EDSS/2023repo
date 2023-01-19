// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  private final CommandPS4Controller m_driverController = 
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);	// This already exists for non-Romi

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Set default command for subsystems
    m_drivetrainSub.setDefaultCommand(new DriveWithJoystickCmd(m_driverController, m_drivetrainSub));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.cross().onTrue(new PrintCommand("Cross pressed!"));

    // This ties a button to a command
    //   m_driverController = The controller who's button you want to use
    //   square() = The button you want to use (others: L1, cross, etc.)
    //   onTrue = When you want the command to trigger (others: whileTrue, toggleOnFalse, etc.)
    //   DriveForwardCmd = The command you want to trigger
    //   (m_drivetrainSub) = The parameters that the command needs
    m_driverController.square().whileTrue(new DriveForwardCmd(m_drivetrainSub));

    // Instead of creating a command for this simple situation, define the command here
    m_driverController.circle().whileTrue(
        new StartEndCommand(
            () -> m_drivetrainSub.tankDrive(-0.25, 0.25),   // Call on command start
            () -> m_drivetrainSub.tankDrive(0.0, 0.0),      // Call on command end
            m_drivetrainSub));                              // Required subsystem
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
