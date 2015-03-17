package LocalNavigation;
/* Based on the pseudo algorithm written by chmike
 * at http://stats.stackexchange.com/questions/23481/are-there-algorithms-for-computing-running-linear-or-logistic-regression-param, Dec 16 '13
 *
 * To use this class: call init before accumulating data. For every new point 
 * acquired, call update(x,y), with x and y in global coordinates. 
 *
 *The equation is then given as y = (Linear)x + (const)
 * and also as: ax+by+c=0 with  b=-1;
 */


public class LinearRegression{
    private double meanX;
    private double meanY;
    private double varX;
    private double varY;
    private double covXY;
    private int n;

    private static final int CORD_X=0;
    private static final int CORD_Y=1;

    public void init(){
        meanX=0;
        meanY=0;
        varX=0;
        varY=0;
        covXY=0;
        n=0;
    }

    public void update(double x, double y){
        n += 1;
        double dx = x - meanX;
        double dy = y - meanY;
        varX += (((n-1.0)/n)*dx*dx - varX)/n;
        covXY += (((n-1.0)/n)*dx*dy - covXY)/n;
        meanX += dx/n;
        meanY += dy/n;
    }
    
    public double getLinearTerm(){
    if (varX==0){
        return 0;
    }
    return covXY/varX;
    }
    
    public double getConstTerm(){
       return meanY - getLinearTerm()*meanX;
    }
    
    public double getA(){
       return getLinearTerm();
    }

    public double getB(){
       return -1.0;
    }

    public double getC(){
       return getConstTerm();
    }

    public double projectPoint(double pointX, double pointY, int cord){
        // two points on the line y = Ax + C: (0,C) and (1,A+C)
        double line0X= 0;
        double line0Y= getConstTerm();
        double line1X= 1;
        double line1Y= line0Y+getLinearTerm();

        double lineLen=Math.sqrt((line1X-line0X)*(line1X-line0X) + (line1Y-line0Y)*(line1Y-line0Y));

        // unit vector pointing from (0,C) to (1,A  +C)
        double lineDirX=0;
        double lineDirY=0;

        if(lineLen!=0){
            lineDirX=(line1X-line0X)/lineLen;
            lineDirY=(line1Y-line0Y)/lineLen;
        }

        pointX=pointX-line0X;
        pointY=pointY-line0Y;

        double projectX= pointX*lineDirX+line0X;
        double projectY= pointY*lineDirY+line0Y;

        if(cord==CORD_X){
            return projectX;
        }
        else {
            return projectY;
        }
    }

    public double projectPointY(double pointX,double pointY){
       return projectPoint(pointX, pointY, CORD_Y);
    }

    public double projectPointX(double pointX,double pointY){
       return projectPoint(pointX, pointY, CORD_X);
    }
    

}
