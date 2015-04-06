package Grasping;
import org.ros.message.rss_msgs.ArmMsg;

public class ShoulderController extends JointController {
    private static final int max = 2350;
    private static final int min = 900;
    
    // private static final int temp = 1600;
    public ShoulderController() {
        super(SHOULDER, (min + max)/2 , min, max);
    }

    public int calculatePWMfromAngle(double angle) {
        //// angle = -.00233*pwm + 3.66779
        //// pwm = (3.66779 - angle)/.00233
    	//angle=-0.00165*pwm+2.61
    	//pwm = (2.61-angle)/0.00165
        //return (int) restrict((3.66779 - angle)/.00233, min, max);
    	return (int) restrict((2.61 - angle)/.00165, min, max);
    }
}
