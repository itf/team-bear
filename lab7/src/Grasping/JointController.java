/**
 * Provides default methods applicable to controlling all types of joints
 */

package Grasping;
import org.ros.message.rss_msgs.ArmMsg;

public abstract class JointController {
    static final int SHOULDER = 0;
    static final int WRIST = 1;
    static final int GRIPPER = 2;
    static final int DEFAULT_SLEW = 5;

    private final int JOINT_TYPE;  // initialized by subclass

    private int slew;
    private int desiredPWM;
    private boolean atGoal;
    private int minPWM;
    private int maxPWM;

    // initialize in here the variables that will vary
    // from subclass to subclass?
    public JointController(int type, int initialPWM, int min, int max) {
        slew = DEFAULT_SLEW;
        atGoal = false;

        JOINT_TYPE = type;
        desiredPWM = initialPWM;

        minPWM = min;
        maxPWM = max;
    }

    // returns a pwm value stepped by slew in the proper direction
    public int handle(ArmMsg m) {
        int lastCommandedPWM = (int) m.pwms[JOINT_TYPE];
        if (lastCommandedPWM < minPWM) lastCommandedPWM = minPWM;
        if (lastCommandedPWM > maxPWM) lastCommandedPWM = maxPWM;
        atGoal = false;

        System.out.println("last commanded: " + lastCommandedPWM + " desired: " + desiredPWM + " slew: " + slew);

        if (lastCommandedPWM < desiredPWM-slew) return lastCommandedPWM + slew;
        if (lastCommandedPWM > desiredPWM+slew) return lastCommandedPWM - slew;
        // else:
        atGoal = true;
        return desiredPWM;
    }

    /**
     * computes using desired angle and known function between servo angle and PWM 
     */
    abstract int calculatePWMfromAngle(double angle);

    public void setDesiredAngle(double angle) {
        this.setDesiredPWM(calculatePWMfromAngle(angle));
    }

    public void setDesiredPWM(int p) {
        this.desiredPWM = p;
    }

    /**
     * TODO: decide visibility of this method. perhaps there are situations
     * where we will need to decide something based on whether 
     * a specific part of the arm is moving?
     */
    public boolean atGoal() {
        return atGoal;
    }

    public static double restrict(double input, int min, int max) {
        if (input>max) return max;
        if (input<min) return min;
        else return input;
    }
}
