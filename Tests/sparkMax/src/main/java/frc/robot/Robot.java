// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

// /**
//  * The VM is configured to automatically run this class, and to call the functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the name of this class or
//  * the package after creating this project, you must also update the build.gradle file in the
//  * project.
//  */
// public class Robot extends TimedRobot {
//   private Command m_autonomousCommand;

//   private RobotContainer m_robotContainer;

//   /**
//    * This function is run when the robot is first started up and should be used for any
//    * initialization code.
//    */
//   @Override
//   public void robotInit() {
//     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//     // autonomous chooser on the dashboard.
//     m_robotContainer = new RobotContainer();
//   }

//   /**
//    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
//    * that you want ran during disabled, autonomous, teleoperated and test.
//    *
//    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//    * SmartDashboard integrated updating.
//    */
//   @Override
//   public void robotPeriodic() {
//     // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//     // commands, running already-scheduled commands, removing finished or interrupted commands,
//     // and running subsystem periodic() methods.  This must be called from the robot's periodic
//     // block in order for anything in the Command-based framework to work.
//     CommandScheduler.getInstance().run();
//   }

//   /** This function is called once each time the robot enters Disabled mode. */
//   @Override
//   public void disabledInit() {}

//   @Override
//   public void disabledPeriodic() {}

//   /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   @Override
//   public void autonomousInit() {
//     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//     // schedule the autonomous command (example)
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.schedule();
//     }
//   }

//   /** This function is called periodically during autonomous. */
//   @Override
//   public void autonomousPeriodic() {}

//   @Override
//   public void teleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.cancel();
//     }
//   }

//   /** This function is called periodically during operator control. */
//   @Override
//   public void teleopPeriodic() {}

//   @Override
//   public void testInit() {
//     // Cancels all running commands at the start of test mode.
//     CommandScheduler.getInstance().cancelAll();
//   }

//   /** This function is called periodically during test mode. */
//   @Override
//   public void testPeriodic() {}

//   /** This function is called once when the robot is first started up. */
//   @Override
//   public void simulationInit() {}

//   /** This function is called periodically whilst in simulation. */
//   @Override
//   public void simulationPeriodic() {}
// }


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftDeviceID = 1; 
  private static final int rightDeviceID = 2;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}