package GlobalNavigation;
import java.awt.geom.Point2D;
import java.util.Iterator;
import java.util.List;

import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;
import org.ros.message.MessageListener;

public class WayPointNavigator {
	
	protected class radialPoint{
		public double theta;
		public double radius;
	}
	
	protected class radialSpeed extends radialPoint{
	}
	
	protected class odoUpdaterListener implements MessageListener<OdometryMsg>{
    	@Override
        public void onNewMessage(OdometryMsg message) {
    		double msgX;
    		double msgY;
    		double msgTheta;
            msgX = message.x;
            msgY = message.y;
            msgTheta = message.theta;

            updateX(msgX);
            updateY(msgY);
            updateTheta(msgTheta);
            updated=true;
    	}

    	protected void updateTheta(double newTheta){
        	//TODO make theta nice?
    		theta=newTheta;
        }
        
        protected void updateX(double newX){
        	x=newX;
        }
        
        protected void updateY(double newY){
        	y=newY;
        }
    }
    
    protected Subscriber<OdometryMsg> odoSub;
    protected Publisher<MotionMsg> motionPub;
    
    //Real localization of the robot
    protected double x;
    protected double y;
    protected double theta;
    protected boolean updated;
    //NEXT POINT/ANGLE. assume that it is always updated
    protected Point2D.Double nextPoint;
    protected double nextTheta;
    protected boolean hasNextPoint;

    //Threshold to when
    protected double distanceThresholdWayPoint=0.2;
    protected double distanceThresholdFinalDestination=0.05;
    
    //Way points
    protected List<Point2D.Double> wayPoints;
    protected Iterator<Point2D.Double> wayPointIterator;

    //Speeds
    protected static final double TRANSLATIONAL_SPEED =0.2;
    protected static final double MAX_ROTATIONAL_SPEED =0.15;
    
	public WayPointNavigator(Node node){
		onStart(node);
	}
    /*
     * run starts the wayPoint navigator, i.e. takes the way points and makes the robot follow them.
     */
    public void run(){
    	while(hasNextPoint){
		    if(updated){
			updated=false;
	    		if (arrivedAtNextPoint(distanceThresholdWayPoint)){
				    updateNextPoint();
				    moveToPoint(nextPoint);
		    	}
	    		else{
	    			moveToPoint(nextPoint);
	    		}
		    }
		    try{
		    	Thread.sleep(10);
		    }catch(Exception e){
		    	
		    }
		    
    	}
    	while (!arrivedAtNextPoint(distanceThresholdFinalDestination)){
    		if(updated){
    			updated=false;
    			moveToPoint(nextPoint);
    		}
		    try{
		    	Thread.sleep(10);
		    }catch(Exception e){
		    	
		    }
    	}
    	setMotion(0, 0);
   		
    }

    public void setWayPoints(List<Point2D.Double> newWayPoints){
    	wayPoints= newWayPoints;
    	wayPointIterator=wayPoints.iterator();
    	updateNextPoint();
    }
    
    protected Point2D.Double updateNextPoint(){
    	if (wayPointIterator.hasNext()){
    		hasNextPoint=true;
    		nextPoint=wayPointIterator.next();
    	}
    	else{
    		hasNextPoint=false;
    	}
    	return nextPoint;
    }
    
    protected void onStart(Node node){
    	motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
    	odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
    	odoSub.addMessageListener(new odoUpdaterListener());
    }
    
    
    protected void moveToPoint(Point2D.Double point){
        //System.out.println("moving to point: " + point);
    	radialPoint pointRobotCoordinate= convertToRobotCoordinates(point);
    	radialSpeed speeds= calculateSpeeds(pointRobotCoordinate);
        //System.out.println("pointRobotCoordinate .theta, .radius:" + pointRobotCoordinate.theta + ", " + pointRobotCoordinate.radius);
        //System.out.println("robotCoordinate .theta:" + theta);

        //System.out.println("distances" + (point.x-x) + "," + ((point.y-y)));
        //System.out.println("speeds.radius: " + speeds.radius + " speeds.theta: " + speeds.theta);
    	setMotion(speeds.radius, speeds.theta);
    }

    
    /*
     * Returns theta between -pi and pi.
     */
    protected radialPoint convertToRobotCoordinates(Point2D.Double point){
    	//TODO test and debug

    	double pointDistance=distanceToRobot(point);
    	
    	double pointTheta=0;
    	double distanceX = point.getX()-x;
    	double distanceY = point.getY()-y;
    	
    	pointTheta=Math.atan2(distanceY, distanceX) - theta; //TODO, check if it is right. 
    										//I don't know how the coordinates are defined in the robot/odometry system
    	while(pointTheta>Math.PI){
    		pointTheta-=2*Math.PI;
    	}
    	while(pointTheta<-Math.PI){
    		pointTheta+=2*Math.PI;
    	}
    	radialPoint robotCoordinates = new radialPoint();
    	robotCoordinates.radius=pointDistance;
    	robotCoordinates.theta=pointTheta;
    	return robotCoordinates;
    }
    
    protected double distanceToRobot(Point2D.Double point){
    	double distanceX = x-point.getX();
    	double distanceY = y-point.getY();
    	
    	double distance = Math.sqrt((distanceX*distanceX) + (distanceY*distanceY));
    	return distance;
    }
    
    protected boolean arrivedAtNextPoint(double distanceThreshold){
    	return distanceToRobot(nextPoint) <= distanceThreshold;
    }
    
    
    /*
     * On this function we take advantage that we have fine control over the robot rotational
     * and translational speed, allowing us to move in archs of circle in the direction of the point
     */
    protected radialSpeed calculateSpeeds(radialPoint desiredDestination) {
    	//TODO test and debug
    	double MAX_DYNAMIC_TURNING_ANGLE = Math.PI/4;
    	double absTheta=Math.abs(desiredDestination.theta);

    	radialSpeed calculatedSpeeds = new radialSpeed();
    	if (absTheta > MAX_DYNAMIC_TURNING_ANGLE){
    		if(desiredDestination.theta>0){
    			calculatedSpeeds.theta=MAX_ROTATIONAL_SPEED*absTheta/Math.PI;
    			calculatedSpeeds.radius=0;
    		}
    		else {
    			calculatedSpeeds.theta=-MAX_ROTATIONAL_SPEED*absTheta/Math.PI;
    			calculatedSpeeds.radius=0;
    		}
    	}
    	
    	else{
    		//Move in an arch of circus in direction of the point
    		double circleRadius = Math.cos(absTheta)/Math.sin(2*absTheta)*desiredDestination.radius; //Geometry
    		double rotationalSpeed= TRANSLATIONAL_SPEED/circleRadius;
    		double translationalSpeed= TRANSLATIONAL_SPEED;
    		if(rotationalSpeed>MAX_ROTATIONAL_SPEED){
    			rotationalSpeed=MAX_ROTATIONAL_SPEED;
    			translationalSpeed=rotationalSpeed*circleRadius;
    		}
    		
    		if(desiredDestination.theta<0){
    			rotationalSpeed=-rotationalSpeed;
    		}
    		
    		calculatedSpeeds.theta=rotationalSpeed;
    		calculatedSpeeds.radius=translationalSpeed;
    		
    	}
    	
    	return calculatedSpeeds;
    }
    
    protected void setMotion(double transVel, double rotVel) {
    	//TODO test and debug
        MotionMsg msg = new MotionMsg();
        msg.translationalVelocity = transVel;
        msg.rotationalVelocity=rotVel;
        motionPub.publish(msg);
    }

    
}
