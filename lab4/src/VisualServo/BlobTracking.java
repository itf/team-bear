package VisualServo;

import java.awt.Color;
import java.util.Arrays;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;


	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()
	public double tv = 0.0;  // set in blobControl()
	public double rv = 0.0;  // set in blobControl()

	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param width image width
	 * @param height image height
	 */
	public BlobTracking(int width, int height) {
		this.width = width;
		this.height = height;
	}

	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			//double fps = (double) stepCounter * 1000.0
			// / (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public void apply(Image src, Image dest) {
		stepTiming(); // monitors the frame rate
		// Begin Student Code
		int pix;
		// System.out.println("R:" + Image.pixelRed(pix) + " G: " 
		// 	+ Image.pixelGreen(pix) + " B: " + Image.pixelBlue(pix));

		int[] sums = new int[3];
		for (int x=this.width/2 - 10; x<this.width/2+10;x++) {
			for (int y=this.height/2 - 10; y<this.height/2+10;y++) {
				pix = src.getPixel(x,y);
				sums[0]+=Image.giveandget(Image.pixelRed(pix));
				sums[1]+=Image.giveandget(Image.pixelGreen(pix));
				sums[2]+=Image.giveandget(Image.pixelBlue(pix));
			}
		}
		// GaussianBlur.apply(src.pixels, dest.pixels, src.getWidth(), src.getHeight());
		// return;

		Histogram.getHistogram(src, dest, true);

		this.blobPresent(src,dest);
		this.blobFix();
		this.blobControl();


		// now highlight the blobpixels:
		int numbp = 0;
		for (int x=0; x<Math.min(256,dest.getWidth());x++) {
			for (int y=0; y<Math.min(256,dest.getHeight());y++) {
				pix = src.getPixel(x,y); // make sure to get the original source pixel!
				if (blobPixel(pix, dest.getHeight())) {
					dest.setPixel(x,y,(byte) 0xff, (byte) 0, (byte) 0xff);
					numbp +=1;
				}
			}
		}

		// // debugging:
		// pix = src.getPixel(src.getWidth()/2,src.getHeight()/2);
		// float[] hsb = Color.RGBtoHSB(Image.giveandget(Image.pixelRed(pix)),
		// 			Image.giveandget(Image.pixelGreen(pix)),
		// 			Image.giveandget(Image.pixelBlue(pix)), 
		// 			null);
		// System.out.println("central pixel: " + Arrays.toString(hsb));

		// highlight centroid
		


		// End Student Code
	}

	// only checks saturation for now
	private boolean blobPixel(int pix, int normfactor) {
		float[] hsb = Color.RGBtoHSB(Image.giveandget(Image.pixelRed(pix)),
					Image.giveandget(Image.pixelGreen(pix)),
					Image.giveandget(Image.pixelBlue(pix)), 
					null);
		return (hsb[0] < 5.0/normfactor && hsb[1] > 50.0/normfactor);  // 5 and 50 is a little too strict I think. 
		// return (hsb[1] > 40) && (hsb[0] < 20);
		// return true;
	}
    //checks for thresholds of hues; 
    //if very red and saturated, return "r"; 
    //if blue and sat, return "b"; 
    //if green and sat, return "g"; 
    //else return "x"
    private char blobPixelColor(int pix, int normfactor) {
	float[] hsb = Color.RGBtoHSB(Image.giveandget(Image.pixelRed(pix)),
				     Image.giveandget(Image.pixelGreen(pix)),
				     Image.giveandget(Image.pixelBlue(pix)),
				     null);
	if (hsb[0]< 5.0/normfactor && hsb[1] > 50.0/normfactor) return 'r'; //should be red only
	else if ((hsb[0] > 90.0/normfactor && hsb[0] < 139) && (hsb[1] > 50.0/normfactor)) return 'g';
	else if ((hsb[0] > 200/normfactor && hsb[0] < 250) && (hsb[1] > 50.0/normfactor)) return 'b';
	else return 'x';
    }

    /* Returns a double[] containing the centroidX and the centroidY of the discovered ball.
       targetArea will be stored in this.targetArea.
     */
	private void blobPresent(Image src, Image dest) {
		int numbp = 0;
		int pix;
		for (int x=0; x<this.width;x++) {
			for (int y=0; y<this.height;y++) {
				pix = src.getPixel(x,y); // make sure to get the original source pixel!
				if (blobPixel(pix, this.height)) {
					dest.setPixel(x,y,(byte) 0xff, (byte) 0, (byte) 0);  // perfectly red
					centroidX += x;
					centroidY += y;
					numbp +=1;
				}
			}
		}

		// should we calculate the centroid even if ball is not present?

		// cutoff value is ball taking up 1% or more of view
		// color centroid with green cross
		// later when we can do conneted components we can detect ball taking up <<1% of the screen i hope
		this.targetArea = 1.0*numbp/(this.width*this.height);
		// System.out.println("target area: " + targetArea);
		if (this.targetArea > 0.01) {
			targetDetected = true;
			centroidX /= numbp;
			centroidY /= numbp;
			// System.out.println("detected w/ centroid: " + centroidX + ", " + centroidY);
			// System.out.println("fix in x-dir: " + (this.width/2-centroidX) + " and in y-dir: " + (centroidY-this.height/2) + "\n");
			for (int x=(int) centroidX-1; x<(int)centroidX+1; x++) {
				for (int y=(int)centroidY-8; y<(int)centroidY+8; y++) {
					if (inRange(x,y)) {
						dest.setPixel(x,y,(byte) 0, (byte) 0xff, (byte) 0);
					}
				}
			}
			for (int x=(int) centroidX-8; x<(int)centroidX+8; x++) {
				for (int y=(int)centroidY-1; y<(int)centroidY+1; y++) {
					if (inRange(x,y)) {
						dest.setPixel(x,y,(byte) 0, (byte) 0xff, (byte) 0);
					}
				}
			}
		} else {
			targetDetected = false;
		}
		return;
	}

	private boolean inRange(int x, int y) {
		return (x>-1 && x<this.width)&&(y>-1 && y<this.height);
	}

	// does some geo to get bearing and distance of ball
    public void blobFix() {
    	if (!this.targetDetected) {
    		return;
    	}
		double testTargetRadius = Math.sqrt(this.targetArea/Math.PI);
		this.targetRange = 0.06/testTargetRadius;
		this.targetBearing = Math.acos(centroidY/Math.sqrt(Math.pow(centroidY,2) + Math.pow(centroidX-this.width/2,2)));
		//check sign of angle; if centroidX is larger than width of image,
		//then angle is negative from the robot origin
		if (centroidX > this.width/2) {
		    this.targetBearing*=-1;
		}
		//System.out.print("targetRange: " + this.targetRange + " tested targetBearing: " + (targetBearing*180/Math.PI));
	       //prints degrees, not radians; target range in m

		return;
    }

    // sets translational and rotational velocity values
    // which are then grabbed in visualservo.java to control the robot
    public void blobControl() {
    	double forwardGain = 0.9;
    	double rotationalGain = 0.20;
    	if (targetDetected) {
    		if (!((this.targetRange-0.5)<0.01 && (this.targetRange-0.5)>-0.01)) {
    			this.tv = (this.targetRange-0.5)*forwardGain;
    		} else {
    			this.tv = 0.0;
    		}

    		if (!(this.targetBearing<1.0/60 && this.targetBearing>-1.0/60)) {
    			this.rv = this.targetBearing*rotationalGain;
    		} else {
    			this.rv = 0.0;
    		}
		
		
    	} else {
    		this.tv = 0.0;
    		this.rv = 0.0;
    	}
    }

  //   /**
  //      Does the transformation from RGB values given to us by the camera back 
  //      to reasonable, human-understandable numbers betwen 0 and 255 inclusive.
  //    */
  //   public int giveandget(byte picol){
		// if (picol < 0) {
		//     return 256 + picol;
		// }
		// else return picol;
	 //   }
}
