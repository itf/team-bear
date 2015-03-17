package GlobalNavigation;

import java.awt.geom.*;
import java.lang.Math;
import java.util.List;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Collections;

public class Cspace {
	public final PolygonObstacle robotPolygon;
	public final List<PolygonObstacle> csObstacles;

	public static final double robotPolygonSides = 20.0;

	
	public Cspace(List<PolygonObstacle> realObstacles, Rectangle2D.Double worldRect, double robotRadius) {
		robotPolygon = new PolygonObstacle();
		for (int i = 0; i< robotPolygonSides; i++) {
			robotPolygon.addVertex(robotRadius*Math.cos(i/robotPolygonSides * 2*Math.PI),
								   robotRadius*Math.sin(i/robotPolygonSides * 2*Math.PI) );
		}

		// realObstacles = (LinkedList) realObstacles;

		PolygonObstacle leftSide = new PolygonObstacle();
		leftSide.addVertex(worldRect.x, worldRect.y);
		leftSide.addVertex(worldRect.x, worldRect.y + worldRect.height);
		System.out.println(realObstacles);
		System.out.println(leftSide);
		realObstacles.add(leftSide);

		PolygonObstacle bottomSide = new PolygonObstacle();
		bottomSide.addVertex(worldRect.x, worldRect.y);
		bottomSide.addVertex(worldRect.x + worldRect.width, worldRect.y);
		realObstacles.add(bottomSide);

		PolygonObstacle rightSide = new PolygonObstacle();
		rightSide.addVertex(worldRect.x + worldRect.width, worldRect.y);
		rightSide.addVertex(worldRect.x + worldRect.width, worldRect.y + worldRect.height);
		realObstacles.add(rightSide);

		PolygonObstacle topSide = new PolygonObstacle();
		topSide.addVertex(worldRect.x, worldRect.y + worldRect.height);
		topSide.addVertex(worldRect.x + worldRect.width, worldRect.y + worldRect.height);
		realObstacles.add(topSide);

		csObstacles = new ArrayList<PolygonObstacle>();
		// compute the configuration space:
		for (PolygonObstacle realObst : realObstacles) {
			csObstacles.add(makeCSObstacle(realObst));
		}
	}
	
	// public java.util.List<PolygonObstacle> getObstacles(){
	// 	return obstacles;
	// }
	protected PolygonObstacle makeCSObstacle(PolygonObstacle realObstacle){
		return computeMinkowskiSum(realObstacle, robotPolygon);
	}
	
	protected PolygonObstacle computeMinkowskiSum(PolygonObstacle polygon1, PolygonObstacle polygon2){
		//Sum = vert(O) ⊕ vert(R)
		List<Point2D.Double> summedPoints = new ArrayList<Point2D.Double>();
		for (Point2D.Double point1: polygon1.getVertices()) {
			for (Point2D.Double point2: polygon2.getVertices()) {
				summedPoints.add(new Point2D.Double(point1.x + point2.x, point1.y + point2.y));
			}
		}

		PolygonObstacle hull = GeomUtils.convexHull(summedPoints);
		return hull;
	}
	
	protected PolygonObstacle reflectInRelationToPoint(PolygonObstacle polygon, Point2D.Double point){
		List<Point2D.Double> reflectedVertices = new ArrayList<Point2D.Double>();
		for (Point2D.Double vertex : polygon.getVertices()) {
			reflectedVertices.add(new Point2D.Double(point.x - vertex.x, point.y - vertex.y));
		}
		PolygonObstacle reflected = new PolygonObstacle();
		Collections.reverse(reflectedVertices);
		for (Point2D.Double ref : reflectedVertices ) {  // make sure to add points in CCW order
			reflected.addVertex(ref);
		}
		return reflected;
	}
	
}
