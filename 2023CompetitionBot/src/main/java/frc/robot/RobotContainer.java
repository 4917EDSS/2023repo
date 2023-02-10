// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToVisionCmd;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.DriveAlignCmd;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.InterruptAllCommandsCmd;
import frc.robot.commands.PickUpCmd;
import frc.robot.commands.DriveSetGearCmd;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.ManipulatorSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.LedSub.LEDMode;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ManipulatorSub.OperationMode;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //private final ManipulatorSub m_manipulatorSub = new ManipulatorSub();
  private final ArmSub m_armSub = new ArmSub();
  private final MastSub m_mastSub = new MastSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final GripperSub m_gripperSub = new GripperSub();
  private final LedSub m_ledSub = new LedSub();
  SendableChooser<Command> m_Chooser = new SendableChooser<>();
  private final VisionSub m_visionSub = new VisionSub(); // Uncomment when
  // limelight connected
  // TODO: Add vision subsystem when camera connected

  // Define controllers
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooserSetup();
    m_visionSub.setPipeline(Constants.LimelightConstants.kApriltag);
    //
    // LedPanell();
    m_ledSub.setColor(-1, 255, 255, 0);

    //

    // Set default command for subsystems
    m_drivetrainSub.setDefaultCommand(new DriveWithJoystickCmd(m_driverController, m_drivetrainSub));
    m_armSub.setDefaultCommand(new ArmMoveWithJoystickCmd(m_operatorController, m_armSub));
    // m_manipulatorSub.setDefaultCommand(new
    // RotateArmWithJoystickCmd(m_driverController, m_manipulatorSub));

    // m_manipulatorSub.setDefaultCommand(new SetArmMastCmd(m_driverController,
    // m_manipulatorSub)); // Testing for arm and mast
    m_armSub.zeroEncoder(); 
    m_mastSub.zeroEncoder(); 
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {
    // Driver controller bindings
    m_driverController.L3().or(m_driverController.R3())
        .whileTrue(new InterruptAllCommandsCmd(m_armSub,m_mastSub, m_gripperSub, m_drivetrainSub));

    m_driverController.povUp().onTrue(new DriveAlignCmd(m_drivetrainSub, m_visionSub, 15.0));

    m_driverController.L1().onTrue(new DriveSetGearCmd(false, m_drivetrainSub));

    m_driverController.R1().onTrue(new DriveSetGearCmd(true, m_drivetrainSub));

    m_driverController.triangle().onTrue(new InstantCommand(() -> m_drivetrainSub.setIsAutoShift(true), // Call on command start
        m_drivetrainSub));

    // Operator controller bindings
    m_operatorController.L3().or(m_operatorController.R3())
        .onTrue(new InterruptAllCommandsCmd(m_armSub,m_mastSub, m_gripperSub, m_drivetrainSub));


    m_operatorController.triangle().onTrue(
        new StartEndCommand(() -> m_gripperSub.setValve(true), () -> m_gripperSub.setValve(true), m_gripperSub));

    m_operatorController.cross().onTrue(new InstantCommand(() -> m_gripperSub.setValve(false), m_gripperSub));


    m_operatorController.square().onTrue(new PickUpCmd(m_armSub,m_mastSub, 0));

    m_operatorController.options().onTrue(new PickUpCmd(m_armSub,m_mastSub, 1));
  }

  void autoChooserSetup() {
    m_Chooser.setDefaultOption("do nothing", getAutonomousCommand());
    m_Chooser.addOption("do nothing2", getAutonomousCommand());
    SmartDashboard.putData("auto choices", m_Chooser);
  }

  // frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
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
    //m_manipulatorSub.resetEncoders();
    m_armSub.zeroEncoder();
    m_mastSub.zeroEncoder();
  }
  /*
   * public void LedPanell () { int r, g, b; r = 0; g = 1; b = 2;
   * 
   * m_ledSub.setLEDState(LEDMode.ConeMode); double[] colour = {Double.valueOf(m_ledSub.m_rr),
   * Double.valueOf(m_ledSub.gg), Double.valueOf(m_ledSub.bb)}; SmartDashboard.putNumberArray("null", colour);
   * 
   * }
   */
}
