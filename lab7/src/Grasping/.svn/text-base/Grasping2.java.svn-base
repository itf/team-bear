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

    private boolean grasping = false;
    private boolean lifting = false;

    private ArmController AC;
    
    @Override
    public void onStart(Node node) {
        System.out.println("starting");
        AC = new ArmController(node);
        // AC.setDesiredPWMs(1575, 1100, 575);
        AC.setAngles3(0,0,0);
        while (!AC.atGoal()) {
            try {Thread.sleep(10);}
            catch(Exception e) {e.printStackTrace();}
        }

        AC.moveUpWithDesiredAngle();
        //AC.setPosition(0.3, 0);
        AC.setPositionRelativeToFloor(0,0);
        try{
        Thread.sleep(5000);
        }
        catch (Exception e){
        	
        }
        AC.setPositionRelativeToFloor(0.1,0);
        try{
        Thread.sleep(5000);
        }
        catch (Exception e){
        	
        }
        AC.setPositionRelativeToFloor(0.2,0);
        try{
        Thread.sleep(5000);
        }
        catch (Exception e){
        	
        }
        AC.setPositionRelativeToFloor(0.3,0);

        // AC.setPosition(0,0,0);

        bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg message) { 
                // if (message.gripper) {
                //     if (!grasping && !lifting) {
                //         AC.closeGripper();
                //         grasping = true;
                //         AC.setShoulderAngle(0);
                //         lifting = true;
                //     }
                // } else {
                //     if (grasping && lifting) {
                //         System.out.println("dropped!");
                //         grasping = false;
                //     } 
                // }
            }   
        });
    }
    
    @Override
    public void onShutdown(Node node) {
        AC.setDesiredPWMs(2100, 550, 900);
    }
    
    @Override
    public void onShutdownComplete(Node node) {
        AC.setDesiredPWMs(2100, 550, 900);
    }
    
    @Override
    public GraphName getDefaultNodeName() {
    return new GraphName("rss/GraspingClass");//?
    }
}