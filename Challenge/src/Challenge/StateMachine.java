package Challenge;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

public class StateMachine  implements NodeMain{

	public class GlobalNavigation implements NodeMain {

	    
	    //CONSTANTs and other class-specific things
	    public static final java.lang.String APPNAME="StateMachine";


	    //publishers
	    //GlobalNavigation needs its own publishers, for messages independent of 
	    //   PolygonMap
	    private Publisher<Object> erasePub;
	    private Publisher<Object> rectPub;
	    private Publisher<Object> polyPub;
	    private Publisher<Object> pointPub;
	    private Publisher<Object> segmentPub;

	    public Publisher<OdometryMsg> resetOdometryPub;


	    /**
	     * Entry hook for ROS when called as stand-alone node. 
	     * Similar to PolygonMap'/s onStart().
	     */
	    @Override
	    public void onStart(Node node) {
	       erasePub = node.newPublisher("/gui/Erase", "lab5_msgs/GUIEraseMsg");
	       pointPub = node.newPublisher("/gui/Point", "lab5_msgs/GUIPointMsg");
	       rectPub = node.newPublisher("/gui/Rect", "lab6_msgs/GUIRectMsg");
	       polyPub = node.newPublisher("/gui/Poly", "lab6_msgs/GUIPolyMsg");
	       segmentPub = node.newPublisher("/gui/Segment", "lab5_msgs/GUISegmentMsg");
	       resetOdometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");

	       this.instanceMain();
	    }

	    public void displayMap() {

	    }

	    @Override
	    public void onShutdown(Node node) {
	    }

	    @Override
	    public void onShutdownComplete(Node node) {
	    }

	    @Override
	    public GraphName getDefaultNodeName() {
	    	return null;  // what else is there to do?
	    }





	    /**
	     * InstanceMain: like a main(). Called in onStart;
	     * */
	    public void instanceMain() {
	    	//Should be an infinite loop
	    		while(false){
	    			
	    		}
	    }
	}

}
