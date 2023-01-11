package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANCoder m_turningEncoder;

  public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId) {
    m_driveMotor = new TalonFX(driveMotorId);
    m_turningMotor = new CANSparkMax(turningMotorId, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_turningEncoder = new CANCoder(turningEncoderId);

    //...
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    //m_driveMotor.set(, 0.0);
    m_turningMotor.set(0.0);
  }

  public void resetEncoders() {
    m_turningEncoder.setPosition(0.0);
  }
}
