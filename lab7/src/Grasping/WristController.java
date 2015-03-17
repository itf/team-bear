package Grasping;
import org.ros.message.rss_msgs.ArmMsg;

public class WristController extends JointController {
    private static final int max = 2100;
    private static final int min = 350;

    // private static final int temp = 1225;

    public WristController() {
        super(WRIST, (min + max)/2, min, max);
    }

    public int calculatePWMfromAngle(double angle) {
        return (int) restrict((2.235 + angle)/.00203, min, max);
    }
}
