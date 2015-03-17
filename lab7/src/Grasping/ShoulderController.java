package Grasping;
import org.ros.message.rss_msgs.ArmMsg;

public class ShoulderController extends JointController {
    private static final int max = 2300;
    private static final int min = 900;
    
    // private static final int temp = 1600;
    public ShoulderController() {
        super(SHOULDER, (min + max)/2 , min, max);
    }

    public int calculatePWMfromAngle(double angle) {
        // angle = -.00233*pwm + 3.66779
        // pwm = (3.66779 - angle)/.00233
        return (int) restrict((3.66779 - angle)/.00233, min, max);
    }
}
