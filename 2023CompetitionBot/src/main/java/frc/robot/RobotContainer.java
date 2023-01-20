// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CylinderCmd;

import frc.robot.subsystems.ManipulatorSub;
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
  // The robot's subsystems and commands are defined here...
  private final ManipulatorSub m_ManipulatorSub = new ManipulatorSub();

  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();  // TODO: Remove example sub when we have one of our own declared

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

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
    m_driverController.circle().onTrue(new PrintCommand("Circle pressed"));

    // Instead of creating a command for this simple situation, define the command here
    m_driverController.cross().onTrue(
      new StartEndCommand(
        () -> m_ManipulatorSub.setSolenoid(true),   // Call on command start
        () -> m_ManipulatorSub.setSolenoid(false),  // Call on command end
        m_ManipulatorSub));                               // Required subsystem
        m_driverController.circle().onTrue(
          new StartEndCommand(
            () -> m_ManipulatorSub.setSolenoid(false),   // Call on command start
            () -> m_ManipulatorSub.setSolenoid(true),  // Call on command end
            m_ManipulatorSub));   
        

    // Set the command to run when no other commands are using this subsystem
    

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);   // TODO: Remove example once we have our own code written
  //}
}
