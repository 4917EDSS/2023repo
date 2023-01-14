
/* 
* NavX/AHRS Hardware Info: 
* https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2019/02/navx-mxp_robotics_navigation_sensor_user_guide.pdf
* Contains Examples, Installations, Software...
*
* API Info:
* NavX communication with MXP SPI (ahrs = new AHRS(SPI.Port.kMXP))
* 
*C:\Users\4917\navx-mxp\java\examples\AutoBalance\src\org\\usfirst\frc\team2465\robot
*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

  AHRS ahrs;

  /** Creates a new NavX. */
  public NavX() 
  {
    ahrs = new AHRS(SPI.Port.kMXP); 
    
    //EXAMPLES//

    //get pitch and roll
    //double pitchAngleDegrees = ahrs.getPitch();
    //double rollAngleDegrees = ahrs.getRoll();

    //get acceleration
    //double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
    //double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
    
    //detect motion
    //boolean motionDetected = ahrs.isMoving();

    //measure rotation
    //double robotRotation = ahrs.getAngle();


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //System.out.print("Yaw = "); 
    //System.out.println(getNavXYaw());
  }

  public double getNavXYaw () {
   return 0;//m_navX.getYaw();
  }
}
