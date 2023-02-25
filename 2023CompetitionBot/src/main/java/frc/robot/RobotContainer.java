// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.DriveAlignCmd;
import frc.robot.commands.DriveAlignTapeCmd;
import frc.robot.commands.DriveSetGearCmd;
import frc.robot.commands.DriveStraightCmd;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.IntakeSetPositionCmd;
import frc.robot.commands.InterruptAllCommandsCmd;
import frc.robot.commands.MastMoveWithJoystickCmd;
import frc.robot.commands.IntakeRotateWithJoystickCmd;
import frc.robot.commands.SetGamePieceTypeCmd;
import frc.robot.commands.StraightenToApriltagCmd;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakePositions;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.LedSub.LedColour;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static Logger logger = Logger.getLogger(RobotContainer.class.getName());
  
  // The robot's subsystems and commands are defined here...
  private final ArmSub m_armSub = new ArmSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final LedSub m_ledSub = new LedSub();
  private final MastSub m_mastSub = new MastSub();
  private final VisionSub m_visionSub = new VisionSub();

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

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

    // Set default command for subsystems
    m_drivetrainSub.setDefaultCommand(new DriveWithJoystickCmd(m_driverController, m_drivetrainSub));
  //  m_armSub.setDefaultCommand(new ArmMoveWithJoystickCmd(m_operatorController, m_armSub));
  //  m_mastSub.setDefaultCommand(new MastMoveWithJoystickCmd(m_operatorController, m_mastSub));
    m_intakeSub.setDefaultCommand(new IntakeRotateWithJoystickCmd(m_operatorController, m_intakeSub));

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
        .onTrue(new InterruptAllCommandsCmd(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub));

    m_driverController.povUp().onTrue(new DriveAlignTapeCmd(m_drivetrainSub, m_visionSub,15.0));

    m_driverController.circle().onTrue(new InstantCommand(() -> m_drivetrainSub.setBrakeCmd(true), m_drivetrainSub));

    m_driverController.cross().onTrue(new InstantCommand(() -> m_drivetrainSub.setBrakeCmd(false), m_drivetrainSub));

    m_driverController.L1().onTrue(new DriveSetGearCmd(false, m_drivetrainSub));

    m_driverController.R1().onTrue(new DriveSetGearCmd(true, m_drivetrainSub));

    m_driverController.triangle().onTrue(
        new InstantCommand(() -> m_drivetrainSub.setIsAutoShift(true), /* Call on command start */ m_drivetrainSub));

    m_driverController.circle().onTrue(new DriveStraightCmd(m_drivetrainSub, 2));

    // Operator controller bindings
    m_operatorController.povUp().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.DOUBLE_STATION));

    m_operatorController.povLeft().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.SINGLE_STATION));

    m_operatorController.povDown().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.GROUND));

    m_operatorController.triangle().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.HIGH));

    m_operatorController.circle().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.MEDIUM));

    m_operatorController.cross().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.LOW));

    m_operatorController.square().onTrue(new InstantCommand(() -> StateOfRobot.m_currentTargetLocation = IntakePositions.START));

    m_operatorController.L1().onTrue(new SetGamePieceTypeCmd(false, this, m_ledSub));

    m_operatorController.R1().onTrue(new SetGamePieceTypeCmd(true, this, m_ledSub));

    m_operatorController.L2().onTrue(m_moveToIntakePositionSelectCmd);
    // The command that runs is dynamically based on the selected position
    m_operatorController.R2().onTrue(m_autoIntakePositionsSelectCmd);

    m_operatorController.PS().onTrue(new InstantCommand(()-> StateOfRobot.m_operatorJoystickforIntake = false));

    m_operatorController.touchpad().onTrue(new InstantCommand(()-> StateOfRobot.m_operatorJoystickforIntake = true));

    m_operatorController.L3().or(m_operatorController.R3())
        .onTrue(new InterruptAllCommandsCmd(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub));
  }

  private SelectCommand m_moveToIntakePositionSelectCmd = new SelectCommand(
    Map.ofEntries(
          Map.entry(IntakePositions.DOUBLE_STATION, new IntakeSetPositionCmd(IntakePositions.DOUBLE_STATION, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.SINGLE_STATION, new IntakeSetPositionCmd(IntakePositions.SINGLE_STATION, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.GROUND, new IntakeSetPositionCmd(IntakePositions.GROUND, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.START, new IntakeSetPositionCmd(IntakePositions.START, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.HIGH, new IntakeSetPositionCmd(IntakePositions.HIGH, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.MEDIUM, new IntakeSetPositionCmd(IntakePositions.MEDIUM, m_armSub, m_mastSub)),
          Map.entry(IntakePositions.LOW, new IntakeSetPositionCmd(IntakePositions.LOW, m_armSub, m_mastSub))),
      this::getTargetLocation);

  private SelectCommand m_autoIntakePositionsSelectCmd = new SelectCommand(
    Map.ofEntries(
          Map.entry(IntakePositions.DOUBLE_STATION, new PrintCommand("Pick-up at double station")),
          Map.entry(IntakePositions.SINGLE_STATION, new PrintCommand("Pick-up at single station")),
          Map.entry(IntakePositions.GROUND, new PrintCommand("Pick-up off of ground")),
          Map.entry(IntakePositions.START, new PrintCommand("Retract mast and arm to starting position")),
          Map.entry(IntakePositions.HIGH, new PrintCommand("Deliver high")),
          Map.entry(IntakePositions.MEDIUM, new PrintCommand("Deliver mid")),
          Map.entry(IntakePositions.LOW, new PrintCommand("Deliver low"))),
      this::getTargetLocation);

  private IntakePositions getTargetLocation() {
      return StateOfRobot.m_currentTargetLocation;
  }

  private void setTargetLocation(IntakePositions targetLocation) {
    StateOfRobot.m_currentTargetLocation = targetLocation;
    System.out.println(targetLocation.name());
  }

  void autoChooserSetup() {
    m_Chooser.setDefaultOption("do nothing", getAutonomousCommand());
    m_Chooser.addOption("do nothing2", getAutonomousCommand());
    SmartDashboard.putData("auto choices", m_Chooser);
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

  public static boolean isConeMode() {
    return StateOfRobot.m_coneMode;
  }

  public static void setConeMode(boolean coneMode) {
    StateOfRobot.m_coneMode = coneMode;
    System.out.println(coneMode ? "Cone Mode" : "Cube Mode");
  }

  public void initSubsystems() {
    m_armSub.init();
    m_drivetrainSub.init();
    m_intakeSub.init();
    m_ledSub.init();
    m_mastSub.init();
    m_visionSub.init();
  }

  public void disabledPeriodic(){
    if(m_intakeSub.isIntakeLoaded()){
      m_ledSub.setColor(0, LedColour.GREEN);
    } else{
      m_ledSub.setColor(0, LedColour.RED);
    }
  }

}
