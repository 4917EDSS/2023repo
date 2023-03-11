package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.subsystems.DrivetrainSub;

public class BalanceChargeStationCmd extends CommandBase {
  private BuiltInAccelerometer mRioAccel;
  private int state;
  private int debounceCount;
  private int count;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double robotSpeedExtraSlow;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;
  private double singleTapTime;
  private double scoringBackUpTime;
  private double doubleTapTime;

  private final DrivetrainSub m_drivetrainSub;


  // TO DO
  //  Add Arm and Mast to home state
  //  Use gyro to drive state.  Add drivestright command to drivetrainsub.

  public BalanceChargeStationCmd(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
    // mRioAccel = new BuiltInAccelerometer();
    state = 0;
    debounceCount = 0;
    count = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot. drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.7;

    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robotSpeedSlow = 0.4;

    // Speed of robot at very end when leveling on the charging station.
    robotSpeedExtraSlow = 0.3;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = 10.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.05;

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
    m_drivetrainSub.arcadeDrive(autoBalanceRoutine(), 0);
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  //routine for automatically driving onto and engaging the charge station.
  //returns a value from -1.0 to 1.0, which left and right motors should be set to.
  public double autoBalanceRoutine() {
    switch(state) {
      //drive forwards to approach station, exit when tilt is detected
      case 0:
        System.out.println("case 0 pitch " + m_drivetrainSub.getPitch());
        if(-m_drivetrainSub.getPitch() > onChargeStationDegree) {
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
        System.out.println("case 1 pitch " + m_drivetrainSub.getPitch());
        if(-m_drivetrainSub.getPitch() < 5) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          System.out.println("2");
          debounceCount = 0;
          return 0;
        }
        count++;
        if(count < secondsToTicks(2)) {
          return 0.5;
        } else {
          return 0.25;
        }
        //on charge station, stop motors and wait for end of auto
      case 2:
        System.out.println("case 2 pitch " + m_drivetrainSub.getPitch());
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
        if(m_drivetrainSub.getPitch() >= 1) {
          System.out.println("-0.2");
          return -robotSpeedExtraSlow;
        } else if(m_drivetrainSub.getPitch() <= -1) {
          System.out.println("0.2");
          return robotSpeedExtraSlow;
        }
      case 3:
        return 0;
    }

    return 0;
  }
}
