package Grasping;
import org.ros.message.rss_msgs.ArmMsg;

public class GripperController extends JointController {
    private static final int max = 1620;
    private static final int min = 200;

    // private static final int temp = 910;


    public GripperController() {
        super(GRIPPER, (min + max)/2, min, max);
    }

    // note: 500 is closed, but below that clamps harder
    // angle: "single" angle (full opening of gripper is 2x single)
    public int calculatePWMfromAngle(double angle) {
        return (int) restrict((angle + 1.04)/.00180, min, max);
    }
}
