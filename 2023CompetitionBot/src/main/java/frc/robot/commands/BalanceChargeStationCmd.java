package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.subsystems.DrivetrainSub;

public class BalanceChargeStationCmd extends CommandBase {
  private BuiltInAccelerometer mRioAccel;
  private int state;
  private int debounceCount;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;
  private double singleTapTime;
  private double scoringBackUpTime;
  private double doubleTapTime;

  private final DrivetrainSub m_drivetrainSub;

  public BalanceChargeStationCmd(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
    // mRioAccel = new BuiltInAccelerometer();
    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.4;

    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robotSpeedSlow = 0.2;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = 10.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.1;

    //Amount of time to drive towards to scoring target when trying to bump the game piece off
    //Time it takes to go from starting position to hit the scoring target
    singleTapTime = 0.4;

    //Amount of time to drive away from knocked over gamepiece before the second tap
    scoringBackUpTime = 0.2;

    //Amount of time to drive forward to secure the scoring of the gamepiece
    doubleTapTime = 0.3;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.zeroHeading();
    m_drivetrainSub.zeroDrivetrainEncoders();
    m_drivetrainSub.setBrake(true);
    m_drivetrainSub.shift(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSub.arcadeDrive(autoBalanceRoutine(), 0);
  }

  // public double getPitch(){
  //     return Math.atan2((- mRioAccel.getX()) , Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
  // }

  // public double getRoll(){
  //     return Math.atan2(mRioAccel.getY() , mRioAccel.getZ()) * 57.3;
  // }

  // //returns the magnititude of the robot's tilt calculated by the root of
  // //pitch^2 + roll^2, used to compensate for diagonally mounted rio
  // public double getTilt(){
  //     if((getPitch() + getRoll())>= 0){
  //         return Math.sqrt(getPitch()*getPitch() + getRoll()*getRoll());
  //     } else {
  //         return -Math.sqrt(getPitch()*getPitch() + getRoll()*getRoll());
  //     }
  // }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  //routine for automatically driving onto and engaging the charge station.
  //returns a value from -1.0 to 1.0, which left and right motors should be set to.
  public double autoBalanceRoutine() {
    switch(state) {
      //drive forwards to approach station, exit when tilt is detected
      case 0:
        if(m_drivetrainSub.getPitch() > onChargeStationDegree) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          System.out.println("1");
          debounceCount = 0;
          return robotSpeedSlow;
        }
        return robotSpeedFast;
      //driving up charge station, drive slower, stopping when level
      case 1:
        System.out.println("case 1");
        if(m_drivetrainSub.getPitch() < levelDegree) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          System.out.println("2");
          debounceCount = 0;
          return 0;
        }
        return robotSpeedSlow;
      //on charge station, stop motors and wait for end of auto
      case 2:
        System.out.println("case 2");
        if(Math.abs(m_drivetrainSub.getPitch()) <= 1) {
          debounceCount++;
          System.out.println("if");
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 4;
          System.out.println("4");
          debounceCount = 0;
          return 0;
        }
        if(m_drivetrainSub.getPitch() >= levelDegree) {
          return -0.2;
        } else if(m_drivetrainSub.getPitch() <= -levelDegree) {
          return 0.2;
        }
      case 3:
        return 0;
    }

    return 0;
  }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSub;

// public class BalanceChargeStationCmd extends CommandBase {
//   private final DrivetrainSub m_drivetrainSub;
//   /** Creates a new BalanceChargeStationCmd. */
//   public BalanceChargeStationCmd(DrivetrainSub drivetrainSub) {
//     m_drivetrainSub = drivetrainSub;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drivetrainSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_drivetrainSub.zeroHeading();
//     m_drivetrainSub.zeroDrivetrainEncoders();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(m_drivetrainSub.getPitch() < 10){

//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
