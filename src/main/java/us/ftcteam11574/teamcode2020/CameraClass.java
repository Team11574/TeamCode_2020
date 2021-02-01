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
    //TODO: take either the average of all centers, to take teh average of centers times teh size of the bounding rectangle
    //TODO: submat to take just certain parts
    //TODO: get locatino based on the image.  
    public double currentTotalColor = 0;
    public double a = 0;
    public double b = 0;
    public double c = 0;


    // Values for mask

    public double hue_low = 0;
    public double hue_high = 0;

    public double sat_low = 0;
    public double sat_high = 0;

    public double value_low = 0;
    public double value_high = 0;

    // Values for BLUR
    int sX = 7;
    int sY = 7;
    public Size blur_size = new Size(sX,sY);




    @Override
    public Mat processFrame(Mat input) { //form of the frame is a matrix
        Mat mat = new Mat();
        if (mat.empty()) { //no image is passed in, this signifies that somethign has gone wrong
            //possibly telemtry that camera is not reciving images
            return input;
        }


        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV); //convert to HSV, and output as mat

        applyBlur(mat);




        //Could write a simple thing to find the amount of yellow between some range, or whether yellow has a sufficently high value within some range

        //note, hue range is normally from 0-360, but openCV keeps colors in the range of 0-180
        //check if the color is in some range
        //Mat res = inRange(mat, 00,40,100,50,160,250);

        Mat rect = new Mat();

        Point left = new Point(
                50,
                50);
        Point right =  new Point(
                100,
                100);
        //input.submat(new Rect(left, right));
        //Imgproc.rectangle(input, left, right, new Scalar(60, 255, 255), 2);



        //Scalar avg = Core.mean(rect);
        //currentTotalColor = avg.val[2];







        //draw a rectangle
        Object[] output = contours(findEdges(mat), mat); //destructive





        int row_s = 0;
        int row_e = 320;
        int col_s = 0;
        int col_e = 240;
        double total_color = 0;
         a = 0;
         b = 0;
        c = 0;
        /*
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



        //Cb.submat(new Rect(region1_pointA, region1_pointB)); this is the region, can use submatrix to figure out the range



        //Mat mat2 = new Mat();
        //Imgproc.cvtColor(mat,mat2,Imgproc.COLOR_HSV2RGB);
        return mat;

    }
    public double returnVal() {
        return currentTotalColor;
    }
    public Mat inRange(Mat mat, int h1, int s1,int v1, int h2, int s2, int v2) {
        Scalar clr1 = new Scalar(h1,s1,v1);
        Scalar clr2 = new Scalar(h2,s2,v2);
        Mat out = new Mat();
        Core.inRange(mat, clr1, clr2, out ); //this makes all the true values black, and all the false values white, desecrtuctive on mat
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
            Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), color, 2);
            //tl() -- top left, br() -- bottom right
            Imgproc.circle(input, centers[i], (int) radius[i][0], color, 2);
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
