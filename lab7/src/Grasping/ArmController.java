package Grasping;

import org.ros.message.rss_msgs.*;
import org.ros.message.MessageListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;


import java.lang.Math;

/**
 * Has control of all the joints, allowing for macro commands like 
 * setPosition
 */
public class ArmController {
    private Publisher<ArmMsg> armPub;
    private Subscriber<ArmMsg> armSub;

    private ShoulderController shoulder;
    private WristController wrist;
    private GripperController gripper;

    public static int counter = 0;

    private static boolean firstLoop = true;

    protected static final double ARM_LENGTH =.2413;
    protected static final double WRIST_LENGTH = .0635 + .1143/3;
    protected static final double SHOULDER_Z = .13335; // shoulder z relative to robot origin
    protected static final double SHOULDER_X = .0762; // shoulder x relative to robot origin
    protected static final double ROBOT_HEIGHT = 0.128;
    //protected static final double MAX_WRIST_ANGLE = Math.PI/2; 
    //protected static final double MIN_WRIST_ANGLE = -Math.PI/2;
    //protected static final double MAX_SHOULDER_ANGLE = Math.PI/2; 
    //protected static final double MIN_SHOULDER_ANGLE = -Math.PI/3; 



    public ArmController(Node node) {
        shoulder = new ShoulderController();
        wrist = new WristController();
        gripper = new GripperController();

        armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
        armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
        armSub.addMessageListener(new MessageListener<ArmMsg> () {
            @Override
            public void onNewMessage(ArmMsg message) {
                if (firstLoop) {
                    firstLoop = false;
                    commandPWMs(2200,1600,575);
                    System.out.println("sleeping for 2000 ms");
                    try {Thread.sleep(2000);}
                    catch(Exception e) {}
                    return;
                }

                // counter +=1;
                // int a = (int) message.pwms[0];
                // int b = (int) message.pwms[1];
                // int c = (int) message.pwms[2];

                // System.out.println("got message #" + counter + ": " + Integer.toString(a) 
                    // + ", " + Integer.toString(b) + ", " + Integer.toString(c));

                int s = shoulder.handle(message);
                int w = wrist.handle(message);
                int g = gripper.handle(message);
                commandPWMs(s, w, g);
            }
        });
    }

    public void setAngles3(double shoulderAngle, double wristAngle, double gripperAngle) {
        setWristAngle(wristAngle);
        setGripperAngle(gripperAngle);
        setShoulderAngle(shoulderAngle);
    }
    public void setAngles3(double shoulderAngle, double wristAngle, double gripperAngle, boolean block) {
        setWristAngle(wristAngle);
        setGripperAngle(gripperAngle);
        setShoulderAngle(shoulderAngle);
        if (block) block("setAngles3: " + wristAngle + "," + gripperAngle + "," + shoulderAngle);
    }

    public void setShoulderAngle(double shoulderAngle) {
        setShoulderAngle(shoulderAngle,false);
    }
    public void setShoulderAngle(double shoulderAngle, boolean block) {
        shoulder.setDesiredAngle(shoulderAngle);
        if (block) block("shoulder angle: " + shoulderAngle);
    }

    public void setWristAngle(double wristAngle) {
        setWristAngle(wristAngle, false);
    }
    public void setWristAngle(double wristAngle, boolean block) {
        wrist.setDesiredAngle(wristAngle);
        if (block) block("wrist angle: " + wristAngle);
    }

    public void setGripperAngle(double gripperAngle) {
        setGripperAngle(gripperAngle, false);
    }
    public void setGripperAngle(double gripperAngle, boolean block) {
        gripper.setDesiredAngle(gripperAngle);
        if (block) block("gripper angle: " + gripperAngle);
    }

    public void openGripper() {
        openGripper(false);
    }
    public void openGripper(boolean block) {
        setGripperAngle(Math.PI/2.5);
        if (block) block("opening gripper.");
    }

    public void closeGripper() {
        closeGripper(false);
    }
    public void closeGripper(boolean block) {
        setGripperAngle(0.1);
        if (block) block("closing gripper.");
    }

    public void block(String debug) {
        int counter = 0;
        while (!this.atGoal()){
            try {counter += 1;
                // System.out.println("not at goal" + counter + ": " + debug);
                Thread.sleep(400);}
            catch(Exception e) {e.printStackTrace();}
        }
        // System.out.println("done blocking: " + debug);
    }

    public void prepareToGrab() {
        // first: keep wrist parallel to ground
        // to do this, simply make sure that shoulder angle is increasing
        // wrist angle is decreasing and equal to the negative of shoulder angle
        int numSteps = 10;
        double theta = -Math.PI/3;
        double theta_step = (Math.PI/3 - theta)/numSteps;
        for (int i = 0; i < numSteps; i ++) {
            theta += theta_step;
            setShoulderAngle(theta,true);
            setWristAngle(-theta,true);

            try {Thread.sleep(5000);}
            catch(Exception e) {}
        }
        
    }

    public void setPosition(double x, double z) {
        double a = .2413; // arm length
        double w = .0635 + .1143/3; // wrist length
        double sh_z = .13335; // shoulder z relative to robot origin
        double sh_x = .0762; // shoulder x relative to robot origin
        double numerator = Math.pow(a+w,2) - Math.pow(x-sh_x,2) - Math.pow(z-sh_z,2); 
        double denominator = Math.pow(x-sh_x,2) + Math.pow(z-sh_z,2) - Math.pow(a-w,2);
        double theta_2 = 2*Math.atan2(Math.sqrt(numerator), Math.sqrt(denominator));
        // elbow down:
        double theta_1 = Math.atan2(z-sh_z, x-sh_x) + Math.atan2(w*Math.sin(theta_2), a+w*Math.cos(theta_2));
        //System.out.println("calculated thetas: " + theta_1 + ", " + theta_2 + " from position: " + x + ", " + z);
        setShoulderAngle(theta_1);
        setWristAngle(theta_2);
        // raise NotImplementedError;
    }
    
    public void setPositionRelativeToFloor(double height, double angleWrist) {
        // calculate z displacement from shoulder
    	height = height - WRIST_LENGTH *Math.sin(angleWrist) -SHOULDER_Z -ROBOT_HEIGHT;
    	if(height>ARM_LENGTH){
    		height=ARM_LENGTH;
    	}
    	if(height<-ARM_LENGTH){
    		height=-ARM_LENGTH;
    	}
        double thetaShoulder = Math.asin(height/ARM_LENGTH);
        // elbow down:
        setShoulderAngle(thetaShoulder);
        setWristAngle(angleWrist-thetaShoulder);
        // System.out.println("calculated thetas: shoulder" + thetaShoulder + ", " + thetaWrist + " wrist ");
        // raise NotImplementedError;
    }

    public void setDesiredPWMs(int s, int w, int g) {
        // System.out.println("setting desired pwms1: " + ", "  + s + ", " + w + ", " + g);
        shoulder.setDesiredPWM(s);
        wrist.setDesiredPWM(w);
        gripper.setDesiredPWM(g);
    }

    private void commandPWMs(int s, int w, int g) {
        ArmMsg cmdMsg = new ArmMsg();
        long[] p=new long[]{0,0,0,0,0,0,0,0};
        p[0] = s;
        p[1] = w;
        p[2] = g;
        cmdMsg.pwms=p;
        //System.out.println("commanding pwms: " + s + ", " + w + ", " + g);
        armPub.publish(cmdMsg);
    }

    public boolean atGoal() {
        return shoulder.atGoal() && wrist.atGoal() && gripper.atGoal();
    }
}