package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

  /**
   * <p>The whole-robot velocity controller.</p>
   **/
  protected RobotVelocityController robotVelocityController;

  /**
   * <p>Total ticks since reset, positive means corresp side of robot moved
   * forward.</p>
   **/
  protected double[] totalTicks = new double[2];

  /**
   * <p>Total elapsed time since reset in seconds.</p>
   **/
  protected double totalTime = 0.0;

  /**
   * <p>Time in seconds since last update.</p>
   **/
  protected double sampleTime;
  

  /**
   * <p>An abstract gain; meaning depends on the particular subclass
   * implementation.</p>
   **/
  protected double gain = 10.0;//1.0;

  /**
   * <p>The robot.</p>
   **/
  protected OdometryRobot robot;
  
  /**
   * <p>Create a new position controller for a robot.</p>
   *
   * @param robot the robot, not null
   **/
  public RobotPositionController(OdometryRobot robot) {
    this.robot = robot;
  }

  /**
   * <p>Translate at the specified speed for the specified distance.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in m/s. positive always (not signed)
   * @param distance the desired distance to move in meters, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean translate(double speed, double distance) {
  	System.out.println("Getting to translate!");
    double LINEAR_TO_ANG_CONVERT = 1/0.063;

  	double TICKS_TO_LINEAR=	this.robotVelocityController.getWheelVelocityController(0).computeRadiansPerTick()*(1/LINEAR_TO_ANG_CONVERT);
  	robot.enableMotors(true);
		boolean ok = true;
    // Begin Student Code
		double initialTickAverage = (totalTicks[0] + totalTicks[1])/2;
		double angSpeed = LINEAR_TO_ANG_CONVERT*speed;//get angular speed from linear speed
		double signedNumTicksToTravel = distance / TICKS_TO_LINEAR;
		double signedError = signedNumTicksToTravel*speed*0.3;

		if (distance < 0){
			angSpeed *= -1;
		}
		
		this.robotVelocityController.setDesiredAngularVelocity(angSpeed,angSpeed);
		
		while (Math.abs((totalTicks[0]+totalTicks[1])/2-initialTickAverage) < Math.abs(signedNumTicksToTravel - signedError)) {	
		// while (Math.abs((totalTicks[0]+totalTicks[1])/2-initialTickAverage) < numTicksToTravel - error) {
			
			//System.out.println("numTicksToTravel is: " + numTicksToTravel);
			//System.out.println("initialTickAverage is: "+ initialTickAverage);
			//System.out.println("totalTicks[0] is: " + totalTicks[0]);
			//System.out.println("totalTicks[1] is: " + totalTicks[1]);
			//pass
		}
		
		this.robotVelocityController.setDesiredAngularVelocity(0,0);
//		robot.enableMotors(false);
		// End Student Code		
	return ok;
  }

  /**
   * <p>Rotate at the specified speed for the specified angle.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in radians/s
   * @param angle the desired angle to rotate in radians, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean rotate(double speed, double angle) {
	  System.out.println("Getting to rotate!");
	    double LINEAR_TO_ANG_CONVERT = 1/0.063;

	  	double TICKS_TO_LINEAR=	this.robotVelocityController.getWheelVelocityController(0).computeRadiansPerTick()*(1/LINEAR_TO_ANG_CONVERT);
	  	robot.enableMotors(true);
		boolean ok = true;
		// Begin Student Code
		double initialTickDifference = totalTicks[1] - totalTicks[0];
		double angSpeed = LINEAR_TO_ANG_CONVERT*speed;//get angular speed from linear speed
		double numTicksToTravel = Math.abs((0.217*angle)/TICKS_TO_LINEAR);
		
		if (angle>0)
			this.robotVelocityController.setDesiredAngularVelocity(-1*angSpeed,angSpeed);
		else if (angle<0)
			this.robotVelocityController.setDesiredAngularVelocity(angSpeed, -1*angSpeed);
		
		while (Math.abs(totalTicks[1]-totalTicks[0]-initialTickDifference) < 2*numTicksToTravel*.985) {
		}
		
		this.robotVelocityController.setDesiredAngularVelocity(0,0);
//
//    double initialTime = totalTime;
//    while (totalTime-initialTime < 6) {
//      
//    }
		//robot.enableMotors(false);
		// End Student Code		
	  return ok;	
  }
    

    

  /**
   * <p>If position control is closed-loop, this computes the new left and
   * right velocity commands and issues them to {@link
   * #robotVelocityController}.</p>
   **/
  public synchronized void controlStep() {

    if (robotVelocityController == null)
      return;

    if (!robot.motorsEnabled() || robot.estopped())
      return;

    // Begin Student Code (if implementing closed-loop control)
    // End Student Code (if implementing closed-loop control)
  }

  /**
   * <p>Set the whole-robot velocity controller.</p>
   *
   * <p>This is called automatically by {@link OdometeryRobot}.</p>
   *
   * @param vc the whole-robot velocity controller
   **/
  public void setRobotVelocityController(RobotVelocityController vc) {
    robotVelocityController = vc;
  }

  /**
   * <p>Set {@link #gain}.</p>
   *
   * @param g the new gain
   **/
  public void setGain(double g) {
    gain = g;
  }

  /**
   * <p>Get {@link #gain}.</p>
   *
   * @return gain
   **/
  public double getGain() {
    return gain;
  }

  /**
   * <p>Update feedback and sample time.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   * @param leftTicks left encoder ticks since last update, positive means
   * corresp side of robot rolled forward
   * @param rightTicks right encoder ticks since last update, positive means
   * corresp side of robot robot rolled forward
   **/
  public synchronized void update(double time,
                                  double leftTicks, double rightTicks) {

    sampleTime = time;

    totalTicks[RobotBase.LEFT] += leftTicks;
    totalTicks[RobotBase.RIGHT] += rightTicks;
    totalTime += time;
  }
}
