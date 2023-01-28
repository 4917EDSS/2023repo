// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.MoveMastCmd;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.ManipulatorSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.ManipulatorSub.ManipulatorMode;

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
  //private final VisionSub m_visionSub = new VisionSub(); // Uncomment when limelight connected
  // TODO: Add vision subsystem when camera connected

  // Define controllers
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
    //     RotateArmWithJoystickCmd(m_driverController, m_manipulatorSub));

    m_manipulatorSub.setDefaultCommand(new MoveMastCmd(m_driverController, m_manipulatorSub));
    m_manipulatorSub.resetEncoders();
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
    // Driver controller bindings
    //m_driverController.povUp().whileTrue(new PrintCommand("Arrow up pressed!!!!!!!"));
    //m_driverController.povDown().whileTrue(new PrintCommand("Arrow down pressed!!!!!!!"));

    /* 
    m_driverController.povLeft().whileTrue(new StartEndCommand( () -> m_manipulatorSub.setMastPosition(0.0),() -> m_manipulatorSub.moveMast(0.0),m_manipulatorSub));
    m_driverController.povUp().whileTrue(new StartEndCommand( () -> m_manipulatorSub.setMastPosition(30.0),() -> m_manipulatorSub.moveMast(0.0),m_manipulatorSub));
    m_driverController.povRight().whileTrue(new StartEndCommand( () -> m_manipulatorSub.setMastPosition(60.0),() -> m_manipulatorSub.moveMast(0.0),m_manipulatorSub));*/


    m_driverController.R1().whileTrue(
      new InstantCommand(
          () -> m_drivetrainSub.shift(true), // Call on command start
          m_drivetrainSub));
    
    m_driverController.L1().whileTrue(
      new InstantCommand(
          () -> m_drivetrainSub.shift(false), // Call on command start
          m_drivetrainSub));

    m_driverController.triangle().onTrue(
      new InstantCommand(
          () -> m_drivetrainSub.autoShift(), // Call on command start
          m_drivetrainSub));

    // Operator controller bindings
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
        new StartEndCommand(
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.6),
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.0), 
            m_manipulatorSub));

    m_operatorController.povLeft().whileTrue(
      new StartEndCommand(
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, -0.6),
            () -> m_manipulatorSub.setManipulatorState(ManipulatorSub.ManipulatorMode.MANUAL, 0.0), 
            m_manipulatorSub));

    m_operatorController.triangle().onTrue(
        new StartEndCommand(
            () -> m_gripperSub.setValve(true), 
            () -> m_gripperSub.setValve(true), 
            m_gripperSub));
            
    m_operatorController.cross().onTrue(
        new InstantCommand(
            () -> m_gripperSub.setValve(false), 
            m_gripperSub));

    m_operatorController.circle().onTrue(
        new InstantCommand(
            () -> m_manipulatorSub.setMastMode(ManipulatorMode.MANUAL,42.9757385253),
              ///42.9757385253,-76.8597106933), 
            m_manipulatorSub)); 
  

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

  public void resetEncoders() {
    m_manipulatorSub.resetEncoders();
  }
}
