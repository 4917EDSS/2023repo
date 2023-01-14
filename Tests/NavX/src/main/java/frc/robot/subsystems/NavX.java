//NavX (aka ahrs) Communication with robot MXP SPI

//C:\Users\4917\navx-mxp\java\examples\AutoBalance\src\org\\usfirst\frc\team2465\robot

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  AHRS m_navX = new AHRS(SPI.Port.kMXP);
  /** Creates a new NavX. */
  public NavX() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.print("Yaw = "); 
//    System.out.println(getNavXYaw());
  }

  public double getNavXYaw () {
   return 0;//m_navX.getYaw();
  }
}
