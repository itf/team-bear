package GlobalNavigation;

import java.awt.geom.*;
import java.io.*;
import java.util.*;
import java.text.*;
import java.awt.Color;

import org.ros.message.lab5_msgs.*;
import org.ros.message.lab6_msgs.*;
import org.ros.message.rss_msgs.*;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

import VisualServo.*;
import LocalNavigation.*;

public class GlobalNavigation implements NodeMain {

    public String myMapFileName=null; //if name is null, pmap can dwi;
    //                            default is: global-nav-maze-2-011-basic.map
    
    //CONSTANTs and other class-specific things
    protected PolygonMap myMap;
    public static final java.lang.String APPNAME="GlobalNavigation";

    public static final double robotRadius = 0.35;
    LinkedList<PolygonObstacle> obstacles;

    //publishers
    //GlobalNavigation needs its own publishers, for messages independent of 
    //   PolygonMap
    private Publisher<Object> erasePub;
    private Publisher<Object> rectPub;
    private Publisher<Object> polyPub;
    private Publisher<Object> pointPub;
    private Publisher<Object> segmentPub;

    public Publisher<OdometryMsg> resetOdometryPub;


    //grid
    public Grid myGrid;
    public Cspace cs;
    private final double GRID_RESOLUTION = 0.01; // meter / #boxes in that m
    
    protected WayPointNavigator wayPointNavigator;

    /**
     * Entry hook for ROS when called as stand-alone node. 
     * Similar to PolygonMap'/s onStart().
     */
    @Override
    public void onStart(Node node) {
       ParameterTree paramTree = node.newParameterTree();
       myMapFileName = paramTree.getString(node.resolveName("~/mapFileName"));

       erasePub = node.newPublisher("/gui/Erase", "lab5_msgs/GUIEraseMsg");
       pointPub = node.newPublisher("/gui/Point", "lab5_msgs/GUIPointMsg");
       rectPub = node.newPublisher("/gui/Rect", "lab6_msgs/GUIRectMsg");
       polyPub = node.newPublisher("/gui/Poly", "lab6_msgs/GUIPolyMsg");
       segmentPub = node.newPublisher("/gui/Segment", "lab5_msgs/GUISegmentMsg");

       resetOdometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
       
       wayPointNavigator = new WayPointNavigator(node);

       this.instanceMain();
    }

    public void displayMap() {
        resetOdometry(resetOdometryPub);
    	//assume Polygonmap has already been instantiated
    	//fill in world rectangle, points (start, goal), and polygons (obstacles) 

    	erasePub.publish(new GUIEraseMsg()); 
    	
    	//world rectangle
    	GUIRectMsg worldRectMsg = new GUIRectMsg(); //BLUE
    	this.fillRectMsg(worldRectMsg, myMap.getWorldRect(), Color.BLUE, false);//unfilled
    	rectPub.publish(worldRectMsg);
    	
    	//robot start position
    	GUIPointMsg robotStartMsg = new GUIPointMsg();
    	this.fillPointMsg(robotStartMsg, myMap.getRobotStart(), Color.GREEN, 1);//cross
    	pointPub.publish(robotStartMsg);
    
    	//robot end position
    	GUIPointMsg robotGoalMsg = new GUIPointMsg();
    	this.fillPointMsg(robotGoalMsg, myMap.getRobotGoal(), Color.RED, 1);//cross
    	pointPub.publish(robotGoalMsg);

    	//PolygonObstacle obstacles
        System.out.println("myMap: " + myMap);
        System.out.println("myMap.getObstacles(): " + myMap.getObstacles());
    	for(PolygonObstacle mapObstacle: myMap.getObstacles()){
            GUIPolyMsg robotObstacMsg = new GUIPolyMsg();
            this.fillPolyMsg(robotObstacMsg, mapObstacle, Color.PINK, false, true);
            polyPub.publish(robotObstacMsg);
        }
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

    public void fillPointMsg(GUIPointMsg msg, Point2D.Double point, Color color, long shape) {
    	//set point message
    	msg.x = point.x;
    	msg.y = point.y;
    	msg.shape = shape;

    	this.fillColor(msg.color,color);
    }

    public void fillSegmentMsg(GUISegmentMsg segmentMsg, Color color, Point2D.Double start, Point2D.Double end) {
    	//set segment Message
    	segmentMsg.startX = start.x;
    	segmentMsg.startY = start.y;
    	segmentMsg.endX = end.x;
    	segmentMsg.endY = end.y;

    	this.fillColor(segmentMsg.color, color);
    }

    public static void fillRectMsg(GUIRectMsg msg, Rectangle2D.Double r, Color c, boolean filled){
    	//set rectangle message to be sent; adjust the msg passed in
    	msg.x = (float) r.x;
    	msg.y = (float) r.y;
    	msg.width = (float) r.width;
    	msg.height = (float) r.height;
    	msg.filled = filled ? 1 : 0;

    	fillColor(msg.c,c);//assumed this.
    }

    public static void fillColor(ColorMsg color2, Color color) {
    	color2.r=color.getRed();
    	color2.g=color.getGreen();
    	color2.b=color.getBlue();
    }

    public static void fillPolyMsg(GUIPolyMsg msg, PolygonObstacle obstacle, Color c, boolean filled, boolean closed) {
    	fillColor(msg.c,c);
    	msg.filled = filled ? 1 : 0; 
        msg.closed = closed ? 1 : 0;

    	LinkedList<Point2D.Double> oVert = (LinkedList) obstacle.getVertices();
    	//filling in x, ys of points; might be neater way to do this?
    	float[] msgx = new float[oVert.size()];
    	float[] msgy = new float[oVert.size()];
    	for (int i = 0; i < oVert.size(); i++){
           Point2D.Double p = oVert.get(i);
           msgx[i] = (float) p.x;
           msgy[i] = (float) p.y;
    	}
    	msg.x = msgx;
    	msg.y = msgy;
    	msg.numVertices = oVert.size();
    }

    public void testConvexHull() {
        List<Point2D.Double> points = new ArrayList<Point2D.Double>();
        points.add(new Point2D.Double(3,0));
        points.add(new Point2D.Double(3,3));
        points.add(new Point2D.Double(1,1));
        points.add(new Point2D.Double(1,5));
        points.add(new Point2D.Double(4,2));
        points.add(new Point2D.Double(6,10));
        points.add(new Point2D.Double(7,8));

        PolygonObstacle hull = GeomUtils.convexHull(points);

        GUIPolyMsg testHullMsg = new GUIPolyMsg();
        this.fillPolyMsg(testHullMsg, hull, Color.PINK, false, true);
        polyPub.publish(testHullMsg);

        GUIPointMsg testHullPointMsg = new GUIPointMsg();
        for (Point2D.Double point: points) {
                this.fillPointMsg(testHullPointMsg, point, Color.RED, 1);//cross
                pointPub.publish(testHullPointMsg);
            } 
    }

    public void makeConfigSpace() {
        cs = new Cspace(myMap.getObstacles(), myMap.getWorldRect(), robotRadius);
        // now plot the cs obstacles
        for (PolygonObstacle csOb : cs.csObstacles) {
            GUIPolyMsg robotObstacMsg = new GUIPolyMsg();
            this.fillPolyMsg(robotObstacMsg, csOb, Color.GREEN, false, true);
            polyPub.publish(robotObstacMsg);
        }
    }

    /**
     * InstanceMain: like a main(). Called in onStart; similar to
     * PolygonMap's instanceMain method. DOES NOT USE PolygonMap's
     * InstanceMain method!! 
     *
     * Roxana wanted to make the main's independent. They should
     * run similarly.
     *
     * Currently, motion and waypoint planning is done within this function.
     * It can be moved, but the touchiness of the computation threads is
     * astounding.
     */
    public void instanceMain() {
        try {
    	    //set PolygonMap
            Thread.sleep(4000);
            myMap = new PolygonMap(new File(myMapFileName));
            this.displayMap();
            //testConvexHull();
            makeConfigSpace();

            myGrid = new Grid(myMap.getWorldRect(), GRID_RESOLUTION);
    	    for(PolygonObstacle csObstacle : cs.csObstacles){
    	       myGrid.markObstacle(csObstacle); // mark cell if part of obstacle
    	    }
    	
    	   System.out.println(myGrid.computeShortestPaths(myMap.getRobotGoal()));
        	
        	//get your cell
        	Grid.Cell whereIAm = myGrid.getCell(myMap.getRobotStart());
        	
        	//List of the points
        	List<Point2D.Double> wayPoints = new LinkedList<Point2D.Double>();
        	while(whereIAm.minDistanceToGoal != 0) { // if still not at goal
        	    Grid.Cell whereToGo = whereIAm.toGoalNext;
        	    
        	    //create new segment message and publish it
        	    GUISegmentMsg pathSection = new GUISegmentMsg();
        	    Point2D.Double whereIAmPoint=new Point2D.Double(whereIAm.getCenterX(), whereIAm.getCenterY());
		    System.out.println("whereIam point"+ whereIAmPoint);
        	    Point2D.Double whereToGoPoint = new Point2D.Double(whereToGo.getCenterX(), whereToGo.getCenterY());
		    System.out.println("wheretogo" + whereToGo);
        	    fillSegmentMsg(pathSection, Color.BLACK, 
        	    		whereIAmPoint, 
        	    		whereToGoPoint);
        	    segmentPub.publish(pathSection);
        	    
        	    wayPoints.add(whereIAmPoint);
        	    whereIAm = whereToGo;
        	}
        	//follow your cell's .toGoalNext to get next cell to goal
        	//Adds point to way point list
        	//publish segment
            
        	//Set the points for the way point navigators
        	wayPointNavigator.setWayPoints(wayPoints);
        	
        	//run the wayPointNavigator
        	wayPointNavigator.run();
    	
        } catch(IOException ioe) {
            ioe.printStackTrace();
        } catch(ParseException pe) {
            pe.printStackTrace();
        } catch(InterruptedException ie) {
            ie.printStackTrace();
        }
    }

    /**
     * Currently empty. Handle's Bump messages
     */
    public void handle(BumpMsg msg) {

    }

    /**
     * Currently empty. Handles Odometry messages
     */
    public void handle(OdometryMsg msg){
    }

    public void resetOdometry(Publisher<OdometryMsg> resetOdometryPub) {
        OdometryMsg resetOdom = new OdometryMsg();
        resetOdom.x=0;
        resetOdom.y=0;
        resetOdom.theta=0;
        resetOdometryPub.publish(resetOdom);        
    }
}
