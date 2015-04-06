package Grasping;

import org.ros.message.rss_msgs.*;
import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

public class Grasping2 implements NodeMain {
    private Subscriber<BumpMsg> bumpSub;
    private Publisher<MotionMsg> motionPub;
    public Subscriber<OdometryMsg> odoSub;

    private Subscriber<ResetMsg> goalFlagSub;
    private Publisher<ResetMsg> droppedPub;

    private boolean grasping = false;
    private boolean lifting = false;

    private final int RAISE_ARM = 0;
    private final int VISUAL_SERVOING = 1;   // sit and wait basically
    private final int GO_TO_PREGRASP = 2;
    private final int FORWARD_TO_BALL = 3;
    private final int GRASP_AND_TRAVEL = 4;
    private final int DROPPED = 5;

    private final int TESTING = 50;  // not handled by any subscribers

    private int state;  

    private ArmController AC;

    // current position of robot in normal cartesian space. updated by odometry listener
    private double x;
    private double y;
    private double theta;

    private double x0;
    private double y0;
    private double theta0;
    
    @Override
    public void onStart(Node node) {
        System.out.println("starting");
        AC = new ArmController(node);
	    // armGymnastics2();

	    state = RAISE_ARM; 
        
        ///////////////////
        state = TESTING;
        System.out.println("opening");
        AC.openGripper(true);
        System.out.println("closing");
        AC.closeGripper(true);
        System.out.println("now moving to different angles");
        AC.setAngles3(Math.PI/2,Math.PI/2,Math.PI/2.5, true);
        System.out.println("shoulder angle pi/2");
        try {Thread.sleep(5000);}
        catch(Exception e) {}
        AC.setAngles3(0,-Math.PI/4,Math.PI/4, true);
        System.out.println("shoulder angle 0");
        try {Thread.sleep(5000);}
        catch(Exception e) {}
        AC.setAngles3(-Math.PI/4,Math.PI/4,Math.PI/8, true);
        System.out.println("shoulder angle -pi/4");
        try {Thread.sleep(5000);}
        catch(Exception e) {}
        ///////////////////


        AC.openGripper(true);
        System.out.println("in starting position");

    droppedPub = node.newPublisher("/rss/Dropped", "rss_msgs/ResetMsg");
    motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
    goalFlagSub = node.newSubscriber("/rss/VisualServoAtGoal", "rss_msgs/ResetMsg");
    goalFlagSub.addMessageListener(new MessageListener<ResetMsg>() {
        @Override
        public void onNewMessage(ResetMsg message) { 
            if (message.reset) {
                state=GO_TO_PREGRASP;
                System.out.println("Aligned to ball, lowering arm");
            }
        }
    });
    
    odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
    odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
        @Override
        public void onNewMessage(OdometryMsg message) {
                x = message.x;
                y = message.y;
                theta = message.theta;
        }     
    });

	bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg message) {
                if (state==RAISE_ARM) {
                    state=VISUAL_SERVOING;
                    AC.setShoulderAngle(Math.PI/2, true);
                    AC.setWristAngle(0.0, true);
                    AC.openGripper(true);
                }
                if (state==GO_TO_PREGRASP) {

                    state = FORWARD_TO_BALL;
                    // Put arm down and open gripper
                    
                    AC.openGripper(true);
                    System.out.println("open gripper");
                    double seventyDegInRad = 1.222;
                    AC.setShoulderAngle(-seventyDegInRad,true);
                    System.out.println("set shoulder angle");
                    AC.setWristAngle(seventyDegInRad, true);
                    System.out.println("set wrist angle");

                    System.out.println("Arm in pre-grasp position");
                    try {Thread.sleep(10000);}
                            catch(Exception e) {e.printStackTrace();}

                    setMotion(0.1,0.0);
                    System.out.println("driving forward toward block");

                }

                if (state==FORWARD_TO_BALL) {
                    if (message.gripper) {
                        setMotion(0.0,0.0);
                        System.out.println("At ball!! closing gripper");
                        AC.closeGripper(true);
                        System.out.println("closed gripper>");
                        System.out.println("<setting shoulder to 0");
                        AC.setShoulderAngle(0, true); 
                        System.out.println("set shoulder to 0>");
                        System.out.println("<setting wrist angle to 0");
                        AC.setWristAngle(0, true); 
                        System.out.println("set wrist angle to 0>");

                        try {Thread.sleep(10000);}
                        catch(Exception e) {e.printStackTrace();}
                        // so now the arm is raised with the ball inside
                        state = GRASP_AND_TRAVEL;
                        // save the current state
                        x0 = x;
                        y0 = y;
                        theta0 = theta;
                        setMotion(0.1,0.00);
                    }
                }            
                if (state==GRASP_AND_TRAVEL) {
                    if (!message.gripper) {
                        System.out.println( "dropped ball" );
                        setMotion(0.0,0.0);
                        ResetMsg droppedMsg = new ResetMsg();
                        droppedMsg.reset = true;
                        droppedPub.publish(droppedMsg);
                        state = RAISE_ARM;

                    } else {
                        double dist = Math.sqrt(Math.pow(x-x0,2) + Math.pow(y-y0,2));
                        System.out.println("distance from x0,y0: " + dist);
                        if (dist >= 0.15) {
                            // have moved far enough
                            setMotion(0.0,0.0);
                            // set the ball down again
                            AC.setPositionRelativeToFloor(0.0,0.0);
                            // try {Thread.sleep(7000);}
                            // catch(Exception e) {e.printStackTrace();}
                            while (!AC.atGoal()){
                                try {System.out.println("not at goal");
                                    Thread.sleep(400);}
                                catch(Exception e) {e.printStackTrace();}
                            }
                            AC.openGripper();
                            System.out.println("deposited ball on ground");

                        }

                    }
                }
            }   
        });

	//create new Visual Servo object; this'll create a BlobTracking object
	// and servo to the ball via motionMsgs sent from VisualServo; other
	// stuff happens behind the scenes.

    }
    
    @Override
    public void onShutdown(Node node) {
        // AC.setDesiredPWMs(2100, 550, 900);
    }
    
    @Override
    public void onShutdownComplete(Node node) {
        // AC.setDesiredPWMs(2100, 550, 900);
    }
    
    @Override
    public GraphName getDefaultNodeName() {
    return new GraphName("rss/GraspingClass");//?
    }

    private void setMotion(double transVel, double rotVel) {
        MotionMsg msg = new MotionMsg();
        msg.translationalVelocity = transVel;
        msg.rotationalVelocity=rotVel;
        motionPub.publish(msg);
    }

    public void armGymnastics2() {
        System.out.println("open gripper");
        AC.openGripper(true);  
        System.out.println("close gripper");
        AC.closeGripper(true);  
        // System.out.println("closed gripper>");
        // System.out.println("<setting shoulder to 0");
        // AC.setShoulderAngle(0, true); 
        // System.out.println("set shoulder to 0>");
        // System.out.println("<setting wrist angle to 0");
        // AC.setWristAngle(0, true); 
        // System.out.println("set wrist angle to 0>");

    }

 //    public void armGymnastics(ArmController AC) {
	// AC.setDesiredPWMs(1575, 1100, 575);
 //        while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //    	AC.openGripper();
 //        try{
 //        Thread.sleep(7000);
 //        }
 //        catch (Exception e){
        	
 //        }
 //        AC.closeGripper();
 //        try{
 //            Thread.sleep(7000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //        //AC.setAngles3(0,0,0);
 //        AC.openGripper();
 //        try{
 //            Thread.sleep(3000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //        AC.setPositionRelativeToFloor(0,0);
 //    	while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //    	try{
 //            Thread.sleep(2000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //        AC.moveUpWithDesiredAngle();
 //        while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //        try{
 //            Thread.sleep(2000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //    	AC.setPositionRelativeToFloor(0.3,1);
 //    	while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //    	try{
 //            Thread.sleep(10000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //    	AC.setPositionRelativeToFloor(0.3,-1);
 //    	while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //    	try{
 //            Thread.sleep(10000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //    	AC.setPositionRelativeToFloor(0.3,1);
 //    	while (!AC.atGoal()) {
 //    		try {Thread.sleep(10);}
 //    		catch(Exception e) {e.printStackTrace();}
 //    	}
 //    	AC.setPositionRelativeToFloor(0,0);
 //    	try{
 //            Thread.sleep(10000);
 //        }
 //        catch (Exception e){
          	
 //        }
 //        /*
 //        double a=0.0;
 //        int i;
 //        for(i=0;i<10;i++){
 //        	System.out.println("height" + a);
 //        	AC.setPositionRelativeToFloor(a,0);
 //            try{
 //            Thread.sleep(10000);
 //            }
 //            catch (Exception e){
            	
 //            }
            
 //            a+=0.02;
            
 //        }
        
 //        for(i=0;i<10;i++){
 //        	System.out.println("height" + a);
 //        	AC.setPositionRelativeToFloor(a,0);
 //            try{
 //            Thread.sleep(10000);
 //            }
 //            catch (Exception e){
            	
 //            }
            
 //            a-=0.02;
            
 //        }
 //    	AC.setPositionRelativeToFloor(0.05,0);
 //         */
 //    }
}