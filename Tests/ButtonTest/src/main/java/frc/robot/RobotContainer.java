// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExamplesCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private static final PS4Controller controller = new PS4Controller(0);

  private final ExamplesCommand m_autoCommand = new ExamplesCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public static void testButtons(){
    if (controller.getL2Button()){
      System.out.println("L2");
    }
    if (controller.getR2Button()){
      System.out.println("R2");
    }
    if (controller.getL1Button()){
      System.out.println("L1");
    }
    if (controller.getR1Button()){
      System.out.println("R1");
    }
    if (controller.getL3Button()){
      System.out.println("L3");
    }
    if (controller.getR3Button()){
      System.out.println("R3");
    }
    if (controller.getSquareButton()){
      System.out.println("Square");
    }
    if (controller.getCrossButton()){
      System.out.println("Cross");
    }
    if (controller.getTriangleButton()){
      System.out.println("Triangle");
    }
    if (controller.getCircleButton()){
      System.out.println("Circle");
    }
    if (controller.getShareButton()){
      System.out.println("Share");
    }
    if (controller.getPSButton()){
      System.out.println("PS");
    }
    if (controller.getOptionsButton()){
      System.out.println("Options");
    }
    if (controller.getTouchpad()){
      System.out.println("Touchpad");
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

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
