package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class AutoDriveOverChargeStationCmd extends CommandBase {
  private int state;
  private int debounceCount;
  private int count;
  private double onChargeStationDegree;
  private double debounceTime;

  private final DrivetrainSub m_drivetrainSub;


  // TO DO
  //  Add Arm and Mast to home state
  //  Use gyro to drive state.  Add drivestright command to drivetrainsub.

  public AutoDriveOverChargeStationCmd(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
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
        if(-m_drivetrainSub.getPitch() > onChargeStationDegree) {
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          return 0.7;
        }
        return 0.9;
      //driving up charge station, drive slower, stopping when level
      case 1:
        if(m_drivetrainSub.getPitch() > 0) {
          state = 2;
        }
        return 0.7;
      //on charge station, stop motors and wait for end of auto
      case 2:
        count++;
        if(count < secondsToTicks(0.8)) {
          return 0.8;
        } else if(count < secondsToTicks(1.4)) {
          return 0.3;
        }
        return 0;
    }

    return 0;
  }
}
