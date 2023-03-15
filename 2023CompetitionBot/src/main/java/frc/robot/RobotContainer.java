// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.AutoBalanceChargeStationCmd;
import frc.robot.commands.AutoConeAndChargeStation;
import frc.robot.commands.AutoConeAndLeaveClose;
import frc.robot.commands.AutoConeAndLeaveFar;
import frc.robot.commands.AutoDoNothingCmd;
import frc.robot.commands.AutoDriveOverChargeStationCmd;
import frc.robot.commands.AutoLeaveAndBalanceGrp;
import frc.robot.commands.DriveAlignTapeCmd;
import frc.robot.commands.DriveSetGearCmd;
import frc.robot.commands.DriveStraightCmd;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.ExpelGamePieceCmd;
import frc.robot.commands.IntakeGamePieceCmd;
import frc.robot.commands.IntakeRotateWithJoystickCmd;
import frc.robot.commands.IntakeSetPositionCmd;
import frc.robot.commands.InterruptAllCommandsCmd;
import frc.robot.commands.MastMoveWithJoystickCmd;
import frc.robot.commands.SetGamePieceTypeCmd;
import frc.robot.commands.SetLimitSwitchesCmd;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.ManipulatorsPositions;
import frc.robot.subsystems.MastSub;
import frc.robot.subsystems.VisionSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static boolean m_brakeMode = true;
  private static boolean m_buttonReady = true;

  private static Logger logger = Logger.getLogger(RobotContainer.class.getName());

  // The robot's subsystems and commands are defined here...
  private final LedSub m_ledSub = new LedSub();
  public final MastSub m_mastSub = new MastSub();
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final ArmSub m_armSub = new ArmSub(m_mastSub, m_intakeSub, m_ledSub);
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
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
    m_armSub.setDefaultCommand(new ArmMoveWithJoystickCmd(m_operatorController, m_armSub));
    m_mastSub.setDefaultCommand(new MastMoveWithJoystickCmd(m_operatorController, m_mastSub));
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

    m_driverController.povUp().onTrue(new DriveAlignTapeCmd(m_drivetrainSub, m_visionSub, 15.0));

    m_driverController.circle().onTrue(new InstantCommand(() -> m_drivetrainSub.setBrakeButton(true), m_drivetrainSub));

    m_driverController.cross().onTrue(new InstantCommand(() -> m_drivetrainSub.setBrakeButton(false), m_drivetrainSub));

    m_driverController.L1().onTrue(new DriveSetGearCmd(false, m_drivetrainSub));

    m_driverController.R1().onTrue(new DriveSetGearCmd(true, m_drivetrainSub));

    m_driverController.triangle().onTrue(
        new InstantCommand(() -> m_drivetrainSub.setIsAutoShift(true), /* Call on command start */ m_drivetrainSub));


    // Operator controller bindings
    m_operatorController.povUp()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.DOUBLE_STATION, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.povLeft()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.SINGLE_STATION, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.povDown()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.GROUND, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.povRight()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.TILTED_GROUND, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.triangle()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.HIGH, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.circle()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.MEDIUM, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.cross()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.LOW, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.square()
        .onTrue(new IntakeSetPositionCmd(ManipulatorsPositions.HOME, m_armSub, m_mastSub, m_intakeSub));

    m_operatorController.L1().onTrue(new SetGamePieceTypeCmd(false, m_ledSub));

    m_operatorController.R1().onTrue(new SetGamePieceTypeCmd(true, m_ledSub));

    // L2 is maped to Intakes in
    m_operatorController.L2().onTrue(new IntakeGamePieceCmd(1.0, m_intakeSub, m_ledSub));

    // R2 is maped to the intake out
    m_operatorController.R2().onTrue(new ExpelGamePieceCmd(0.5, m_intakeSub));

    m_operatorController.options().onTrue(new SetLimitSwitchesCmd(m_mastSub, m_armSub, m_intakeSub));

    m_operatorController.PS().onTrue(new InstantCommand(() -> StateOfRobot.m_operatorJoystickforIntake = false));

    m_operatorController.touchpad().onTrue(new InstantCommand(() -> StateOfRobot.m_operatorJoystickforIntake = true));

    m_operatorController.L3().or(m_operatorController.R3())
        .onTrue(new InterruptAllCommandsCmd(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub));
  }

  void autoChooserSetup() {
    m_Chooser.setDefaultOption("1 do nothing", new AutoDoNothingCmd());
    m_Chooser.addOption("2 drive straight", new DriveStraightCmd(m_drivetrainSub, 3));
    m_Chooser.addOption("3 expel game piece", new ExpelGamePieceCmd(1.0, m_intakeSub));
    m_Chooser.addOption("4 balance on Charge Station", new AutoBalanceChargeStationCmd(m_drivetrainSub, true));
    m_Chooser.addOption("5 Drive Over Charge Station", new AutoDriveOverChargeStationCmd(m_drivetrainSub, true));
    m_Chooser.addOption("6 Leave and Balance", new AutoLeaveAndBalanceGrp(m_drivetrainSub));
    m_Chooser.addOption("7 Score Cone, Leave and Balance",
        new AutoConeAndChargeStation(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub, m_ledSub));
    m_Chooser.addOption("8 Score Cone and Leave Far",
        new AutoConeAndLeaveFar(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub, m_ledSub));
    m_Chooser.addOption("9 Score Cone and Leave Close",
        new AutoConeAndLeaveClose(m_armSub, m_mastSub, m_intakeSub, m_drivetrainSub, m_ledSub));

    SmartDashboard.putData("auto choices", m_Chooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_Chooser.getSelected();
  }

  public void initSubsystems() {
    m_armSub.init();
    m_drivetrainSub.init();
    m_intakeSub.init();
    m_ledSub.init();
    m_mastSub.init();
    m_visionSub.init();
  }

  public void initTests() {
    m_armSub.initTest();
    m_intakeSub.initTest();
    m_mastSub.initTest();
  }

  public void disabledPeriodic() {
    // Manually disable motor brakes
    if((HALUtil.getFPGAButton() == true) && m_buttonReady) {

      m_brakeMode = !m_brakeMode;
      m_drivetrainSub.setBrake(m_brakeMode);
      m_mastSub.setBrake(m_brakeMode);
      m_armSub.setBrake(m_brakeMode);
      m_intakeSub.setBrake(m_brakeMode);

      logger.finest("Brake " + m_brakeMode);
      m_buttonReady = false;
    }
    if(HALUtil.getFPGAButton() == false) {
      m_buttonReady = true;
    }

    // MAST
    m_ledSub.setZoneRGB(LedZones.DIAG_MAST_ENC, 0, (int) (m_mastSub.getPosition() / 18.0 * 255), 0);
    if(m_mastSub.isMastAtLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_MAST_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_MAST_LIMIT, LedColour.RED);
    }


    // ARM
    if(m_armSub.getPosition() > 0) {
      m_ledSub.setZoneRGB(LedZones.DIAG_ARM_ENC, 0, (int) (m_armSub.getPosition() / 24000.0 * 255), 0);
    } else if(m_armSub.getPosition() < 0) {
      m_ledSub.setZoneRGB(LedZones.DIAG_ARM_ENC, (int) (m_armSub.getPosition() / -20000.0 * 255), 0, 0);
    } else {
      m_ledSub.setZoneRGB(LedZones.DIAG_ARM_ENC, 0, 0, 0);
    }

    // INTAKE
    m_ledSub.setZoneRGB(LedZones.DIAG_INTAKE_ENC, 0, (int) m_mastSub.getPosition() / 6 * 255, 0);

    // Show sensor and encoder status on LEDs when the robot isn't enabled
    if(m_intakeSub.isIntakeLoaded()) {
      m_ledSub.setZoneColour(LedZones.DIAG_INTAKE_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_INTAKE_LIMIT, LedColour.RED);
    }
  }
}
