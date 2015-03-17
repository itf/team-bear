package LocalNavigation;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.MotionMsg;

import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.io.*;
import java.nio.charset.*;
import java.util.*;
import java.text.*;

public class LocalNavigation implements NodeMain {

    // private static final double sonar_base = 0.25; // distance from fron sonar to back sonar

    private int state;
    
    public static final int STOP_ON_BUMP = 0;
    public static final int ALIGN_ON_BUMP = 1;
    public static final int ALIGNING = 2;
    public static final int ALIGNED = 3;  // both bumpers touching wall

    public static final int ALIGNED_AND_BACKING = 4;
    public static final int ALIGNED_AND_STOPPED = 5;
    public static final int ALIGNED_AND_TURNING = 6; 
    public static final int ALIGNED_AND_STOPPED_AGAIN = 7; // when parallel to wall at distance d

    public static final int BACKING_UP = 9;
    public static final int FINDING_WALL = 10;
    public static final int TRACKING_WALL = 11;
    public static final int WALL_ENDED = 12;

    public static final int SEARCHING_FOR_NEXT_WALL = 13;
    public static final int DONE = 14;

    public static final int TRANSITIONING = 15;
    public int nextState;
    // used for making the robot stop for a few loops between actions
    public int loopCounter;
    public int loopsToWait;
    public boolean firstLoop; // used during transitions to execute an action on the first loop of new state

    public static final double SLOW_SPEED = 0.03;
    public static final double MEDIUM_SPEED = 0.12;

    // the sonar can see long ranges; this is just the cutoff value (in meters) at which point we stop
    // considering the sonar readings as corresponding to the object we care about
    public double obstacleCutoff;
    public double desired_d_from_wall;
    public double CIRCLE_RADIUS;

    public static boolean saveErrors;
    private static Charset UTF8 = Charset.forName("UTF-8");

    public Subscriber<OdometryMsg> odoSub;
    public Subscriber<SonarMsg> frontSonarSub;
    public Subscriber<SonarMsg> backSonarSub;
    public Subscriber<BumpMsg> bumpSub;

    public Publisher<org.ros.message.std_msgs.String> statePub;
    public Publisher<MotionMsg> motionPub;
    public Publisher<OdometryMsg> resetOdometryPub;

    public Publisher<GUILineMsg> guiLinePub;
    public Publisher<GUISegmentMsg> guiSegmentPub;
    public Publisher<GUIEraseMsg> guiErasePub;
    public Publisher<GUIPointMsg> guiPointPub;

    public LinearRegression linReg;

    protected boolean firstUpdate;

    // current position of robot in normal cartesian space. updated by odometry listener
    public double x;
    public double y;
    public double theta;

    public double realTheta; // not mod 2pi

    // position of robot after first wall alignment
    public boolean firstAlignment = false;
    public double firstX;
    public double firstY;
    public double firstRealTheta;

    // saved position for backing and turning specific amounts
    public boolean savedPosition = false;
    public double x0;
    public double y0;
    public double theta0;
    public double desiredTheta;

    public boolean debug;

    // only use the values from lastFront and lastBack when
    // detectedFront and detectedBack are true
    // those booleans are set when the range is less than the cutoff value
    // i.e. detectedFrontObstacle
    public boolean detectedFront;
    public boolean detectedBack;

    public double lastFrontRange; // don't need to initialize these 
    public double lastBackRange; 

    public GUIPointMsg lastFrontPoint;
    public GUIPointMsg lastBackPoint;

    GUISegmentMsg wall;
    private boolean savedStart = false;
    private boolean savedEnd = false;
    /**
     * <p>Create a new LocalNavigation object.</p>
     */
    public LocalNavigation() {
       setInitialParams();
    }
    
    protected void setInitialParams() {
        saveErrors=true; //save errors, etc to file "errorsFile.txt"
        firstUpdate = true;
        debug = true;
        loopCounter = 0;

        wall = new GUISegmentMsg();  // where the current wall segment coordinates are stored

        lastFrontPoint = new GUIPointMsg();  // where the last point in range of the sonars is stored
        lastBackPoint = new GUIPointMsg();

        detectedFront = false;
        detectedBack = false;

        desired_d_from_wall= 0.3;
        obstacleCutoff = desired_d_from_wall*1.15;
        
        CIRCLE_RADIUS = Math.sqrt(Math.pow(desired_d_from_wall+8.5*.0254,2) + Math.pow(9.25*.0254,2)); 
        System.out.println("desired d from wall: " + desired_d_from_wall + " circle radius: " + CIRCLE_RADIUS);   
        state = SEARCHING_FOR_NEXT_WALL;
    }

    @Override
    public void onStart(Node node) {
        motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
        statePub = node.newPublisher("/rss/state", "std_msgs/String");
        guiPointPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");
        guiLinePub = node.newPublisher("gui/Line",  "lab5_msgs/GUILineMsg");
        guiSegmentPub = node.newPublisher("gui/Segment",  "lab5_msgs/GUISegmentMsg");
        resetOdometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");

        linReg= new LinearRegression();
        linReg.init();

        odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
        odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
            @Override
            public void onNewMessage(OdometryMsg message) {
                checkCompletion();

                odometryStateHandler(message);
                publishState();
            }
        });

        frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
        frontSonarSub.addMessageListener(new MessageListener<SonarMsg>() {
            @Override
            public void onNewMessage(SonarMsg message) {
                checkCompletion();

                updateSonarData(message, true); 
                publishLinReg();
                sonarStateHandler(message, true);
                publishState();
            }
        });
        
        backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
        backSonarSub.addMessageListener(new MessageListener<SonarMsg>() {
            @Override
            public void onNewMessage(SonarMsg message) {
                checkCompletion();

                updateSonarData(message, false); 
                publishLinReg();
                sonarStateHandler(message, false);
                publishState();
            }       
        });
        
        bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg message) { 
                checkCompletion();

                bumpStateHandler(message);            
                publishState();   
            }   
        });
    }
    
    @Override
    public void onShutdown(Node node) {
        if (node != null) {
            node.shutdown();
        }
    }
    
    @Override
    public void onShutdownComplete(Node node) {
    }
    
    @Override
    public GraphName getDefaultNodeName() {
    return new GraphName("rss/local_nav");
    }

    public void odometryStateHandler(OdometryMsg message) {
        if (firstUpdate) {
            firstUpdate = false;
            setMotion(0.0, 0.0);
            resetOdometry(resetOdometryPub);
            return;
        } 

        updateRobotConfiguration(message);

        // state handling:
        if (state==ALIGNED_AND_BACKING) {
            if (!savedPosition) { // initial position. used to calculate relative displacement
                x0 = x;
                y0 = y;
                theta0 = theta;
                savedPosition = true;
            } 

            double dist = Math.sqrt(Math.pow((x0-x),2) + Math.pow((y0-y),2));
            if (dist >= desired_d_from_wall*0.95) {  // 0.95 is just an error factor
                savedPosition = false;
                state = ALIGNED_AND_STOPPED;  
            } else {
                // System.out.println("backing away from wall. currently at distance: " + dist);
                setMotion(-MEDIUM_SPEED,0.0); 
            }
        }

        if (state==ALIGNED_AND_STOPPED) {
            waitThenTransition(ALIGNED_AND_TURNING,10);
        }

        if (state==ALIGNED_AND_TURNING) {
            if (!savedPosition) {
                x0 = x;
                y0 = y;
                theta0 = theta;
                savedPosition = true;
                desiredTheta = theta0 - Math.PI/2; 
                if (desiredTheta < 0) desiredTheta += 2*Math.PI;
            }

            // // be careful: theta does not update very fast, so need a fairly large error range
            // assumed that theta is decreasing during the turn
            if (Math.abs(theta - desiredTheta) < 0.15) {  // 0.15 is the error range
                setMotion(0.0,0.0);
                savedPosition = false;
                state = ALIGNED_AND_STOPPED_AGAIN;
            } else {
                // System.out.println("turning. theta, theta0, desiredtheta: " + theta + ", " + theta0 + ", " + desiredTheta);
                setMotion(0.0,-MEDIUM_SPEED);
            }
        }

        if (state==ALIGNED_AND_STOPPED_AGAIN) {
            if (!firstAlignment) {
                firstX = x;
                firstY = y;
                firstRealTheta = realTheta;
                firstAlignment = true;
                System.out.println("saved first alignment position: firstX, firstY, firstRealTheta: " + firstX + "," + firstY+ "," + firstRealTheta);
            }
            
            waitThenTransition(BACKING_UP, 10);
        }

        if (state == TRANSITIONING) {
            transitionHandler();
        }
    }

    public void bumpStateHandler(BumpMsg message) {
        if (state==STOP_ON_BUMP) {
            if (message.left || message.right) {
            setMotion(0.0,0.0);
            }
        } else if (state==ALIGN_ON_BUMP) {
            if (message.left || message.right) {
                state = ALIGNING;
            }
        } else if (state==SEARCHING_FOR_NEXT_WALL) {
            if (message.left || message.right) {
                System.out.println("found next wall! aligning now");
                state = ALIGNING;
            } else {
                double speed = MEDIUM_SPEED;
                double rotSpeed = speed/CIRCLE_RADIUS/3.0;
                // System.out.println("searching for next wall. moving with tv, rv: " + speed + ", " + rotSpeed);
                setMotion(speed, rotSpeed);
            }
        }
        
        // this will execute even if the state was just changed
        if (state==ALIGNING) {
            if (message.left && message.right) {
                state = ALIGNED;
                debug = false;
                waitThenTransition(ALIGNED_AND_BACKING, 10);
            } else if (message.left) {
                // System.out.println("left bumper. rotating cc");
                // robot rotates counterclockwise
                setMotion(0.0, SLOW_SPEED);
            } else if (message.right) {
                // System.out.println("right bumper. rotating ccw");
                // robot rotates clockwise
                setMotion(0.0,-SLOW_SPEED);
            } else {
                // move forward REALLY slowly
                // System.out.println("neither bumper. moving forward slowly");
                setMotion(SLOW_SPEED,0.0);
            }
        }

        if (state==ALIGNED) {
            System.out.println("ALIGNED!");
            waitThenTransition(ALIGNED_AND_BACKING, 10);
        }
        
        if (state == TRANSITIONING) {
            transitionHandler();
        }
    }

    public void sonarStateHandler(SonarMsg message, boolean isFrontSonar) {
        // follow the wall while moving backwards
        if (state==BACKING_UP) {
            if (firstLoop) {
                linReg.init(); 
                firstLoop = false;
                return;
            }
            if (!detectedFront && !detectedBack) {
                System.out.println("transitioning to finding wall!");
                waitThenTransition(FINDING_WALL,20);
            } else {
                // proportional controller to keep robot parallel to wall
                // while moving backwards
                if (detectedFront && detectedBack) {
                    if (saveErrors) logErrors("./savedErrorsBackingUp.txt");

                    // should turn to try to minimize difference between sonar readings
                    // positive rv means ccw means decreasing front range and increasing back range
                    setMotion(-MEDIUM_SPEED,1.5*(lastFrontRange-lastBackRange));
                } else {
                    setMotion(-MEDIUM_SPEED,0.0);
                }
            }
        }

        if (state==FINDING_WALL) {
            if (firstLoop) {
                linReg.init();
                firstLoop = false;
                return; 
            }
            if (detectedFront || detectedBack) {
                if (!savedStart) {
                    // save the point that caused the transition
                    wall.startX = lastFrontPoint.x;
                    wall.startY = lastFrontPoint.y;
                    savedStart = true;  
                    System.out.println("saving wall start point: " + wall.startX + "," + wall.startY);


                    GUIPointMsg wallStart = new GUIPointMsg();
                    wallStart.x = wall.startX;
                    wallStart.y = wall.startY;
                    ColorMsg wallStartColor = new ColorMsg();
                    wallStartColor.r=255;
                    wallStartColor.b=255;
                    wallStartColor.g=0;
                    wallStart.shape = 2;
                    guiPointPub.publish(wallStart);
                }

                waitThenTransition(TRACKING_WALL,20);                
            } else {
                // slowly walk forward
                setMotion(SLOW_SPEED, 0.0);
            }
        }

        if (state==TRACKING_WALL) {
            if (firstLoop) {
                linReg.init();
                firstLoop = false;
                return;
            } 

            if (!detectedFront && !detectedBack) {
                if (!savedEnd) {
                    // // BAD!! saves the frontsonar point, which is long past
                    // save the point that caused the transition
                    // wall.endX = point.x;
                    // wall.endY = point.y;
                    wall.endX = lastBackPoint.x;
                    wall.endY = lastBackPoint.y;
                    savedEnd = true;
                    System.out.println("saving wall end point: " + wall.endX + "," + wall.endY);

                    GUIPointMsg wallEnd = new GUIPointMsg();
                    wallEnd.x = wall.endX;
                    wallEnd.y = wall.endY;
                    ColorMsg wallEndColor = new ColorMsg();
                    wallEndColor.r=0;
                    wallEndColor.b=0;
                    wallEndColor.g=255;
                    wallEnd.shape = 2;
                    guiPointPub.publish(wallEnd);
                }
                waitThenTransition(WALL_ENDED,20);
            } else {
                // proportional controller to keep robot parallel to wall 
                // while going forwards
                double rv;
                if (saveErrors) logErrors("./savedErrorsTrackingWall.txt");

                if (detectedFront && detectedBack) {
                    // should turn to try to minimize difference between sonar readings
                    // positive rv means ccw means decreasing front range and increasing back range
                    setMotion(MEDIUM_SPEED,1.5*(lastFrontRange-lastBackRange));
                } else {
                    setMotion(MEDIUM_SPEED,0.0);
                }
            }
        }

        if (state==WALL_ENDED) {
            // assumes that the wall endpoints have been saved in the GUISegmentMsg wall
            System.out.println("entered state WALL_ENDED");
            System.out.println("measured wall start x,y: " + wall.startX + "," + wall.startY);
            System.out.println("measured wall ended x,y: " + wall.endX + "," + wall.endY);

            ColorMsg wallColor = new ColorMsg(); 
            // wallColor.r = (int) Math.random()*256;
            // wallColor.g = (int) Math.random()*256;
            // wallColor.b = (int) Math.random()*256;
            wallColor.r = 255;
            wallColor.g = 102;
            wallColor.b = 0;
            wall.color = wallColor;

            double tempX =wall.startX;
            double tempY =wall.startY;
            wall.startX=linReg.projectPointX(tempX,tempY);
            wall.startY=linReg.projectPointY(tempX,tempY);

            tempX =wall.endX;
            tempY =wall.endY;
            wall.endX=linReg.projectPointX(tempX,tempY);
            wall.endY=linReg.projectPointY(tempX,tempY);
            System.out.println("projected wall start x,y: " + wall.startX + "," + wall.startY);
            System.out.println("projected wall end   x,y: " + wall.endX + "," + wall.endY);

            guiSegmentPub.publish(wall);

            // also publish some more segments nearby so that they're visible on map
            GUISegmentMsg wall2 = new GUISegmentMsg();
            wall2.color = wallColor;

            double wallLength = Math.sqrt(Math.pow(wall.endX-wall.startX, 2) + Math.pow(wall.endY-wall.startY, 2));
            double wall2offset = 0.02;
            double perpY = -(wall.endX - wall.startX)/wallLength * wall2offset;
            double perpX = (wall.endY - wall.startY)/wallLength * wall2offset;

            wall2.startX = wall.startX + perpX;
            wall2.endX = wall.endX + perpX;
            wall2.startY = wall.startY + perpY;
            wall2.endY = wall.endY + perpY;
            guiSegmentPub.publish(wall2);
            System.out.println("wall2 start x,y: " + wall2.startX + "," + wall2.startY);
            System.out.println("wall2 end   x,y: " + wall2.endX + "," + wall2.endY);
            wall2.startX = wall.startX - perpX;
            wall2.endX = wall.endX - perpX;
            wall2.startY = wall.startY - perpY;
            wall2.endY = wall.endY - perpY;
            guiSegmentPub.publish(wall2);
            System.out.println("wall2 prime start x,y: " + wall2.startX + "," + wall2.startY);
            System.out.println("wall2 prime end   x,y: " + wall2.endX + "," + wall2.endY);

            wall = new GUISegmentMsg();

            state = SEARCHING_FOR_NEXT_WALL;
            linReg.init();
        }
    }

    public void logErrors(String filename) {
        try {
            Writer writer = new OutputStreamWriter(new FileOutputStream(filename), UTF8);
            DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
            Date date = new Date();
            writer.write(dateFormat.format(date)); //2014/08/06 15:59:48. example from mkyong.com
            writer.write(" " + ((lastBackRange+lastFrontRange)/2) + " " + Math.atan((lastBackRange-lastFrontRange)/0.26035));//translation error, rotation error
            writer.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }

    // the waiting is NOT done inside this method. instead uses dangerous global variables!
    public void waitThenTransition(int next, int loops) {
        setMotion(0.0,0.0);
        state = TRANSITIONING;
        nextState = next;
        loopCounter = 0;
        loopsToWait = loops;
        firstLoop = true;
    } 

    public void transitionHandler() {
        if (loopCounter > loopsToWait) {
            state = nextState;
        } else {
            loopCounter +=1;
            // System.out.println("waiting to transition to state: " + nextState + " loopCounter: " + loopCounter);
        }
    }

    public void checkCompletion() {
        if (completedTour()){
            state=DONE;
            setMotion(0.0,0.0);
            System.out.println("DONE!!");
        }
    }

    // completion condition
    public boolean completedTour() {
        if (realTheta > firstRealTheta + 2*Math.PI - 0.2) {
            System.out.println("have completed full revolution. firstRealTheta, realTheta: " + firstRealTheta + "," + realTheta);
            if (Math.sqrt(Math.pow(x-firstX,2) + Math.pow(y-firstY,2)) < desired_d_from_wall) {
                System.out.println("within " + desired_d_from_wall + " of first alignment position."); 
                System.out.println("done!");
                return true;
            }
        }
        return false;
    }


    public void updateRealTheta(double messageTheta) {
        // old/current theta is theta
        // new theta is message.theta
        // assume no more than pi radians of rotation between updates:
        double partition = theta - Math.PI;
        if (partition<0) partition+=2*Math.PI;

        if (theta<=Math.PI) {
            if (messageTheta<=theta && messageTheta>=0) realTheta+=messageTheta-theta;
            else if (messageTheta>theta && messageTheta<partition) realTheta+=messageTheta-theta;
            else if (messageTheta>=partition) realTheta+=(messageTheta-2*Math.PI)-theta;
            else System.out.println("you messed up. theta, realTheta, messageTheta: " + theta + ", " + realTheta + ", " + messageTheta);
        } else { // theta > Math.PI
            if (messageTheta<=theta && messageTheta>=partition) realTheta+=messageTheta-theta;
            else if (messageTheta>theta) realTheta+=messageTheta-theta;
            else if (messageTheta<partition) realTheta+=(messageTheta+2*Math.PI)-theta;
            else System.out.println("you messed up. theta, realTheta, messageTheta: " + theta + ", " + realTheta + ", " + messageTheta);
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////
    ////// pretty confident in the below functions. probably shouldn't have to modify ///////
    /////////////////////////////////////////////////////////////////////////////////////////

    public void updateRobotConfiguration(OdometryMsg message) {
        x = message.x;
        y = message.y;
        updateRealTheta(message.theta);
        updateTheta(message.theta);
    }           

    public void updateTheta(double messageTheta) {
        theta = messageTheta;
    }

    public void setMotion(double transVel, double rotVel) {
        MotionMsg msg = new MotionMsg();
        msg.translationalVelocity = transVel;
        msg.rotationalVelocity=rotVel;
        motionPub.publish(msg);
    }
    
    public void publishState() {
        String temp = "new state: " + state;
        org.ros.message.std_msgs.String message = new org.ros.message.std_msgs.String();
        message.data = temp;
        statePub.publish(message);
    }

    public void updateSonarData(SonarMsg message, boolean isFrontSonar) {
        GUIPointMsg point;
        if (isFrontSonar) {
            point = calculateAndPublishPoint(message, 5.7*0.0254, isFrontSonar);
        } else {
            point = calculateAndPublishPoint(message, -7.5*0.0254, isFrontSonar);
        }

        if (message.range<obstacleCutoff) {
            linReg.update(point.x, point.y);
            if (isFrontSonar) {
                detectedFront = true;
                lastFrontRange = message.range;
                lastFrontPoint.x = point.x;
                lastFrontPoint.y = point.y;
                // if (state == ALIGNED_AND_STOPPED_AGAIN || (state>=9&&state<=12)) 
                    // System.out.println("front sonar: " + message.range);
            } else {
                detectedBack = true;
                lastBackRange = message.range;
                lastBackPoint.x = point.x;
                lastBackPoint.y = point.y;
                // if (state == ALIGNED_AND_STOPPED_AGAIN || (state>=9&&state<=12)) 
                    // System.out.println("back sonar: " + message.range);
            }

        } else {
            if (isFrontSonar) {
                detectedFront = false;
                // System.out.println("front sonar over cutoff: " + message.range);

            } else {
                detectedBack = false;
                // System.out.println("back sonar over cutoff: " + message.range);
            }
        }
    }

    // does the trig to calculate the point's location
    // displacement is the distance (in m) from the sonar sensor to the origin of the robot along the (robot's) x-axis
    // also returns the calculated point for use elsewhere
    public GUIPointMsg calculateAndPublishPoint(SonarMsg msg, double displacement, boolean colorBlue) {
        GUIPointMsg point = new GUIPointMsg();
        point.x = x + displacement*Math.cos(theta) + (0.20 + msg.range)*Math.cos(theta+Math.PI/2);
        point.y = y + displacement*Math.sin(theta) + (0.20 + msg.range)*Math.sin(theta+Math.PI/2);
        point.shape = 0;

        ColorMsg pointCol = new ColorMsg();

        if (msg.range<obstacleCutoff) { 
            // blue for back sonar, red for front
            if (colorBlue) {
                pointCol.r=0;
                pointCol.b=255;
                pointCol.g=0;
            } else {
                pointCol.r=255;
                pointCol.b=0;
                pointCol.g=0;
            }
        }
        else { // green
            pointCol.r = 0;
            pointCol.g = 255;
            pointCol.b = 0;
        }
        point.color = pointCol;
        guiPointPub.publish(point);
        return point;
    }

    public void publishLinReg(){
        GUILineMsg lineMsg = new GUILineMsg();
        lineMsg.lineA= linReg.getA();
        lineMsg.lineB= linReg.getB();
        lineMsg.lineC= linReg.getC();

        ColorMsg lineColor = new ColorMsg();
        lineColor.r = 0;
        lineColor.g = 127;
        lineColor.b = 0;

        lineMsg.color = lineColor;
        // System.out.println("publishing line: " + lineMsg.lineA + ", " + lineMsg.lineB + ", " + lineMsg.lineC); 
        guiLinePub.publish(lineMsg);
    }

    public void resetOdometry(Publisher<OdometryMsg> resetOdometryPub) {
        OdometryMsg resetOdom = new OdometryMsg();
        resetOdom.x=0;
        resetOdom.y=0;
        resetOdom.theta=0;
        resetOdometryPub.publish(resetOdom);        
    }


}
