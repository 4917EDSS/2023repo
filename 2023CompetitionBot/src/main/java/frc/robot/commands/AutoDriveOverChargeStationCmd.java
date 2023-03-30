package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class AutoDriveOverChargeStationCmd extends CommandBase {
  private int state;
  private int debounceCount;
  private int count;
  private double onChargeStationDegree;
  private double debounceTime;
  private boolean m_isForward;
  private boolean m_isFinished = false;

  private final DrivetrainSub m_drivetrainSub;


  // TO DO
  //  Add Arm and Mast to home state
  //  Use gyro to drive state.  Add drivestright command to drivetrainsub.

  public AutoDriveOverChargeStationCmd(DrivetrainSub drivetrainSub, boolean isForward) {
    m_drivetrainSub = drivetrainSub;
    m_isForward = isForward;
    addRequirements(drivetrainSub);
    // mRioAccel = new BuiltInAccelerometer();
    state = 0;
    debounceCount = 0;
    count = 0;

    /**********
     * CONFIG *
     **********/
    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.05;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    debounceCount = 0;
    count = 0;
    m_drivetrainSub.zeroHeading();
    m_drivetrainSub.zeroDrivetrainEncoders();
    m_drivetrainSub.setBrake(true);
    m_drivetrainSub.shift(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isForward) {
      System.out.println("Forward");
      m_drivetrainSub.arcadeDrive(autoDriveOverStation(-1), 0);
    } else {
      System.out.println("Reverse");
      m_drivetrainSub.arcadeDrive(-autoDriveOverStation(1), 0);
    }

  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  //routine for automatically driving onto and engaging the charge station.
  //returns a value from -1.0 to 1.0, which left and right motors should be set to.
  public double autoDriveOverStation(double direction) {
    switch(state) {
      //drive forwards to approach station, exit when tilt is detected
      case 0:
        System.out.println("Case 0: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        if(direction * m_drivetrainSub.getPitch() > onChargeStationDegree) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          return 0.6;
        }
        return 0.6;

      case 1:
        System.out.println("Case 1: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        if(-direction * m_drivetrainSub.getPitch() > 0) {
          state = 2;
        }
        return 0.6;
      //on charge station, stop motors and wait for end of auto
      case 2:
        System.out.println("Case 2: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        count++;
        if(count < secondsToTicks(0.5)) {
          return 0.6;
        } else if(count < secondsToTicks(0.8)) {
          return 0.5;
        }
        m_isFinished = true;
        return 0;
    }
    return 0;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_isFinished) {
      return true;
    }
    return false;
  }
}
