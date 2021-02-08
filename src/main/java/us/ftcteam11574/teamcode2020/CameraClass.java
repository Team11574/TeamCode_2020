package us.ftcteam11574.teamcode2020;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//PLAN:
    //Can sample different region and look at what they seem like to figure out the edges stuff
    //

public class CameraClass extends OpenCvPipeline {

    //TODO: include telemetry so I can test this stuff out on sunday.
    //TODO: cut image to just a certain size
    //TODO: get some coutour stuff working to recognize the location
        //Contour: https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        //Create a bounding box baseed on tehe colors, and find the height of the bounding box
    //TODO: can rewrite everything into the YCrCB channel, to avoid light issues
    //TODO: take either the average of all centers, to take teh average of centers times the size of the bounding rectangle
    //TODO: submat to take just certain parts
    //TODO: get locatino based on the image.  
    
    
    //NOTE: all the current values used in this aren't tested. I haven't yet worked out the correct search area, or optimized parameters for the hue, saturation, and value
    public double currentTotalColor = 0;
    public double a = 0;
    public double b = 0;
    public double c = 0;
    
    public Scalar out_yellow = new Scalar(0,0,0);
    public Scalar out_yellow_small = new Scalar(0,0,0);
    
    
    
    //search area for the the stacks. Looks at the full stack of up to 4 rings
    Point left_search = new Point(
                160-40,
                120-80);
    Point right_search =  new Point(
                160+40,
                120+80);
    
    //A second search can make it easier to see the difference between 0 or 1 items inside the stack
    Point small_left_search = new Point(
                160-40,
                120-40);
    Point small_right_search =  new Point(
                160+40,
                120+40);
    
    //Yellow Hue
    public int Yhue = 15; //remember, only from 0-180 NOT 0-360
    //25 seems fairly optimized
    public int Ysat = 200;
    //This is not very optimized
    public int Yval = 100;
    //


    // Values for mask, just yellow versus some mask
    
    public int[] thresh = new int[]{10,55,155};

    public int hue_low = Yhue - thresh[0];
    public int hue_high = Yhue + thresh[0];

    public int sat_low = Ysat - thresh[1];
    public int sat_high = Ysat + thresh[1];

    public int value_low = Yval - thresh[2];
    public int value_high = Yval + thresh[2];

    // Values for BLUR
    int sX = 7; //must be an odd value
    int sY = 7; //must be an odd value
    public Size blur_size = new Size(sX,sY);




    @Override
    public Mat processFrame(Mat input) { //form of the frame is a matrix
        Mat mat = new Mat();
        if (input.empty()) { //no image is passed in, this signifies that somethign has gone wrong
            //possibly telemtry that camera is not reciving images
            out_yellow = new Scalar(-1,-1,-1);
            return input;
        }


        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV); //convert to HSV, and output as mat


        applyBlur(input); //apply a blur to make it easier to recognize contours
        
        input = inRange(input, hue_low,sat_low,value_low,hue_high,sat_high,value_high); //find the pixels within some range, and keep those. Returns a grey scale image


        Mat stack_mat = input.submat(new Rect(left_search, right_search)); //This gets a sub matrix within a certain range, which should be where the stack is. Taking the average of the
        //colors should give a good estimation of the number of items within the stack
        
        Mat zero_v_one_mat = input.submat(new Rect(small_left_search, small_right_search)); //This gets the sub matrix of the bottom ring.
        
        out_yellow = Core.mean(stack_mat); //Not entirely sure if this function is correctly, but it shoudl take the mean within that range. The lighter it is, the more yellow must of been there
        out_yellow_small = Core.mean(zero_v_one_mat); // ^ same as above.


        
        
        
        
        
        



        






        try {
            Mat res = findEdges(input);

            Object[] output = contours(res, input); //destructive, creates the contours, and saves them to the Object class, this is mainly useful for seeing bounding
            //boxes. This probably won't be all that useful later
        }
        catch (Exception e) {
            //bad form, for testing only
        }




        /*
        int row_s = 0;
        int row_e = 320;
        int col_s = 0;
        int col_e = 240;
        double total_color = 0;
         a = 0;
         b = 0;
        c = 0;
        
        for (int r = row_s;row_e > r;r++ ) {
            for (int cl = col_s;col_e > c;c++ ) {
                a += mat.get(r,cl)[0];
                b += mat.get(r,cl)[1];
                c += mat.get(r,cl)[2];

            }
        }

        a /=  ( (row_e-row_s) * (col_e-col_s));
        b /=  ( (row_e-row_s) * (col_e-col_s));
        c /=  ( (row_e-row_s) * (col_e-col_s));
        */
        
        
        return input;

    }
    public double returnVal() {
        return currentTotalColor;
    }
    public Scalar[] returnYellows() {
        return new Scalar[]{out_yellow, out_yellow_small};
    }
    public Mat inRange(Mat mat, int h1, int s1,int v1, int h2, int s2, int v2) {
        Scalar clr1 = new Scalar(h1,s1,v1);
        Scalar clr2 = new Scalar(h2,s2,v2);
        Mat out = new Mat();
        Core.inRange(mat, clr1, clr2, out ); //this makes all the true values black, and all the false values white, desecrtuctive on mat. 
        //Values that are close get lighter, and farther away values are darker
        return out;
        //example: https://stackoverflow.com/questions/36693348/java-opencv-core-inrange-input-parameters
    }
    public void applyBlur(Mat input) {
        Imgproc.GaussianBlur(input,input, blur_size,sX,sY);
    }
    public Mat findEdges(Mat image) {
        Mat edges = new Mat();
        Imgproc.Canny(image, edges, 255/3, 255); //destructive
        return edges; //this returns the edges as an image
        //example: https://docs.opencv.org/master/da/d22/tutorial_py_canny.html
    }
    public Object[] contours(Mat edges, Mat input) {
        //TODO: Return a struct that has a better shape

        //currently, this draws directly to the original image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(); //hierarchy of countours
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //RETR_TREE returns the entire heirarchy of countours. Things inside of others thing will be childrene of what they are inside of



        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];


        for (int i = 0; i < contours.size(); i++) { //each contour is its own closed shape
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                //needs to be closed, and epsilon is the accuracy of the polygons.
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray())); //draws a boundign rectangle
            centers[i] = new Point(); //saves the centers
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]); //creates an enclosing circle
        }

        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(155, 155, 155);
            Imgproc.drawContours(input, contoursPolyList, i, color);

            //tl() -- top left, br() -- bottom right
            if(radius[i][0] > 20) {
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), color, 2);
                Imgproc.circle(input, centers[i], (int) radius[i][0], color, 2);
            }
        }

        Object[] res = new Object[]{centers, boundRect, radius};
        return res;



    }
    public ArrayList<Mat> toYCrCb(Mat input) {

        Mat tmp = new Mat();
        Imgproc.cvtColor(input,tmp,Imgproc.COLOR_RGB2YCrCb); //by default, the image is in RGB, converts to YCrCB
        ArrayList<Mat> res= new ArrayList<Mat>(3);
        Core.split(tmp, res);

        return res;

    }






}
