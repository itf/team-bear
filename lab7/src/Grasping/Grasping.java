package Grasping;

import org.ros.message.rss_msgs.*;
import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

/**
 * Grasping, the class :
 * For PART 2/checkpoint:
 * (1) receives arm messages. method handle(ArmMsg msg) moves each servo 
 *     through its full range of motion, moving all servos concurrently (and smoothly?)
 * (2) handle() method repeats the motion indefinitely - requires a state machine
 * BEWARE: don't move servos in too wild a range of motion, and pair them well.
 * BEWARE: servos should move at most 1 RADIAN per iteration. 
 *
 * For PART 3:
 * (1) should open-grip
 * (2) should close grip
 * (3) should move up or down with a desired angle - make sure that arm angle is not below 
 *     the floor threshold.
 * (4) bend elbow (wrist? ) with a desired angle
 * (5) move the gripper to ground: alpha = lower threshold, omega = -alpha
 */
public class Grasping implements NodeMain {

    private Publisher<ArmMsg> armPub;
    private Subscriber<ArmMsg> armSub;

    public final int REST = 0;
    public final int HOKEYPOKEYIN = 1;
    public final int HOKEYPOKEYOUT = 2;

    public int state = HOKEYPOKEYIN;
	
    @Override
    public void onStart(Node node) {
	armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
	armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
	armSub.addMessageListener(new MessageListener<ArmMsg> (){
		@Override
		public void onNewMessage(ArmMsg message) {
		    handle(message);//
		}
	    });
    }
    
    @Override
    public void onShutdown(Node node) {
    }
    
    @Override
    public void onShutdownComplete(Node node) {
    }
    
    @Override
    public GraphName getDefaultNodeName() {
	return new GraphName("rss/GraspingClass");//?
    }
    
    /**
     * The arm message contains 8 floats, and the 
     * first three control the servos of the arm.
     * 
     * handle receives an armstatus ArmMsg, and then tells the arm, via 
     * a publisher, to do something relative to its status;
     */
    public void handle(ArmMsg msg) {
	double shoulderMove = msg.pwms[0];
	double wristMove = msg.pwms[1];
	double gripperMove = msg.pwms[2];
	
	System.out.println("\n\n\n\nstate is: " + state + "\n\n\n");
	ArmMsg cmdMsg = new ArmMsg();
	long[] p=new long[8];
	if (state==HOKEYPOKEYIN){
	    p[0]= 1200;// big?
	    p[1] = 1200;//wrist?
	    p[2] = 1200;//gripper?
	    state = HOKEYPOKEYOUT;
	}
	else if (state==HOKEYPOKEYOUT){
	    p[0]= 1000;// big?
	    p[1] = 1000;//wrist?
	    p[2] = 1000;//gripper?
	    state = HOKEYPOKEYIN;
	}
	p[3] = 0; //others...
	p[4] = 0;
	p[5] = 0;
	p[6] = 0;
	p[7] = 0;
	cmdMsg.pwms=p;
	armPub.publish(cmdMsg);
    
    }

}