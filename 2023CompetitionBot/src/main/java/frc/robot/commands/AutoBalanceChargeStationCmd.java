package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class AutoBalanceChargeStationCmd extends CommandBase {
  private int state;
  private int debounceCount;
  private int count;
  private double onChargeStationDegree;
  private double debounceTime;
  private boolean m_isForward;
  private long m_timeStart;
  //private boolean m_isFinished = false;

  private final DrivetrainSub m_drivetrainSub;


  // TO DO
  //  Add Arm and Mast to home state
  //  Use gyro to drive state.  Add drivestright command to drivetrainsub.

  public AutoBalanceChargeStationCmd(DrivetrainSub drivetrainSub, boolean isForward) {
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
    debounceTime = 0.1;

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

    m_timeStart = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isForward) {
      System.out.println("Balance Forward");
      m_drivetrainSub.arcadeDrive(autoBalanceRoutine(-1), 0);
    } else {
      System.out.println("Balance Backward");
      m_drivetrainSub.arcadeDrive(-autoBalanceRoutine(1), 0);
    }

  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  //routine for automatically driving onto and engaging the charge station.
  //returns a value from -1.0 to 1.0, which left and right motors should be set to.
  public double autoBalanceRoutine(double direction) {
    switch(state) {
      //drive forwards to approach station, exit when tilt is detected
      case 0:
        //System.out.println("Balance Case 0: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        if((direction * m_drivetrainSub.getPitch()) > onChargeStationDegree) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          return 0.35;
        }
        if(RobotController.getFPGATime() - m_timeStart > 3200000) {
          return 0.5;
        } else if(RobotController.getFPGATime() - m_timeStart > 2000000) {
          return -0.2;
        }
        return 0.6;
      //driving up charge station, drive slower, stopping when level
      case 1:
        //System.out.println("Balance Case 1: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        if((direction * m_drivetrainSub.getPitch()) < 5) {

          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          debounceCount = 0;
          return 0;
        }
        count++;
        if(count < secondsToTicks(1.8)) {
          return 0.35;
        } else {
          return 0.25;
        }
        //on charge station, stop motors and wait for end of auto
      case 2:
        System.out.println("Case 2: Dir: " + direction + " Pitch: " + m_drivetrainSub.getPitch());
        if(Math.abs(m_drivetrainSub.getPitch()) <= 1) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 3;
          debounceCount = 0;
          return 0;
        }
        if((direction * m_drivetrainSub.getPitch()) <= -2) {
          return -0.225;
        } else if((direction * m_drivetrainSub.getPitch()) >= 2) {
          return 0.225;
        } else {
          m_drivetrainSub.setBrake(true);
          return 0;
        }
      case 3:
        //m_isFinished = true;
        m_drivetrainSub.setBrake(true);
        return 0;
    }
    return 0;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0, 0);
    m_drivetrainSub.setBrake(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if(m_isFinished) {
     * return true;
     * }
     */
    return false;
  }
}
