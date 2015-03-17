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
                    commandPWMs(1575,1100,575);
                    try {Thread.sleep(500);}
                    catch(Exception e) {}
                    firstLoop = false;
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

    public void setAngles3(int shoulderAngle, int wristAngle, int gripperAngle) {
        setWristAngle(wristAngle);
        setGripperAngle(gripperAngle);
        setShoulderAngle(shoulderAngle);
    }

    public void setShoulderAngle(double shoulderAngle) {
        shoulder.setDesiredAngle(shoulderAngle);
    }
    public void setWristAngle(double wristAngle) {
        wrist.setDesiredAngle(wristAngle);
    }
    public void setGripperAngle(double gripperAngle) {
        gripper.setDesiredAngle(gripperAngle);
    }

    public void openGripper() {
        setGripperAngle(7*Math.PI/6);
    }
    public void closeGripper() {
        setGripperAngle(0.1);
    }

    public void moveUpWithDesiredAngle() {
        // first: keep wrist parallel to ground
        // to do this, simply make sure that shoulder angle is increasing
        // wrist angle is decreasing and equal to the negative of shoulder angle
        int numSteps = 10;
        double theta = -Math.PI/3;
        double theta_step = (Math.PI/3 - theta)/numSteps;
        for (int i = 0; i < numSteps; i ++) {
            theta += theta_step;
            setShoulderAngle(theta);
            setWristAngle(-theta);

            while (!atGoal()) {
                try {Thread.sleep(10);}
                catch(Exception e) {}
            }
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
        System.out.println("calculated thetas: " + theta_1 + ", " + theta_2 + " from position: " + x + ", " + z);
        setShoulderAngle(theta_1);
        setWristAngle(theta_2);
        // raise NotImplementedError;
    }
    
    public void setPositionRelativeToFloor(double height, double angleWrist) {
    	height = height - WRIST_LENGTH *Math.sin(angleWrist);
        double thetaShoulder = Math.asin((height-SHOULDER_Z-ROBOT_HEIGHT)/ARM_LENGTH);
        // elbow down:
        setShoulderAngle(thetaShoulder);
        double thetaWrist=angleWrist-thetaShoulder; //+constant
        setWristAngle(thetaWrist);
        System.out.println("calculated thetas: shoulder" + thetaShoulder + ", " + thetaWrist + " wrist ");
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
        System.out.println("commanding pwms: " + s + ", " + w + ", " + g);
        armPub.publish(cmdMsg);
    }

    public boolean atGoal() {
        return shoulder.atGoal() && wrist.atGoal() && gripper.atGoal();
    }
}