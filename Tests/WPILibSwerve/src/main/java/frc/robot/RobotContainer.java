// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSub;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
    DrivetrainSub m_drivetrainSub = new DrivetrainSub();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS4Controller m_driverController =
            new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        m_drivetrainSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_drivetrainSub.drive(
                                m_driverController.getLeftY(),
                                m_driverController.getLeftX(),
                                m_driverController.getRightX(),
                                true),
                        m_drivetrainSub));
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
        m_driverController.cross().onTrue(new PrintCommand("Cross pressed!"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        //  Generate trajectory             
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.1, 0),
                        new Translation2d(0.2, 0.2)),
                new Pose2d(0.5, 0.2, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // Define PID Controller for tracking trajectory
        ProfiledPIDController thetaController =
                new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                        AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        trajectory,
                        m_drivetrainSub::getPose, // Functional interface to feed supplier
                        DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0),
                        thetaController,
                        m_drivetrainSub::setModuleStates,
                        m_drivetrainSub);

        // An example command will be run in autonomous
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSub.resetOdometry(trajectory.getInitialPose()), m_drivetrainSub),
                swerveControllerCommand,
                new InstantCommand(() -> m_drivetrainSub.stopModules(), m_drivetrainSub));
    }
}
