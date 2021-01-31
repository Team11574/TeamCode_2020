package us.ftcteam11574.teamcode2020;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
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
    public double currentTotalColor = 0;
    @Override
    public Mat processFrame(Mat input) { //form of the frame is a matrix
        Mat mat = new Mat();
        //Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2YCrCb); //by default, the image is in RGB, converts to YCrCB
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) { //no image is passed in, this signifies that somethign has gone wrong
            //possibly telemtry that camera is not reciving images
            return input;
        }

        //Could write a simple thing to find the amount of yellow between some range, or whether yellow has a sufficently high value within some range

        //note, hue range is normally from 0-360, but openCV keeps colors in the range of 0-180
        //check if the color is in some range
        input = inRange(input, 20,100,100,40,255,255);
        Mat rect = new Mat();
        Point left = new Point(
                50,
                50);
        Point right =  new Point(
                100,
                100);
        rect.submat(new Rect(left, right));

        Scalar avg = Core.mean(rect);
        currentTotalColor = avg.val[2];







        //draw a rectangle
        Imgproc.rectangle(
                input, // Buffer to draw on
                left, // First point which defines the rectangle
                right, // Second point which defines the rectangle
                new Scalar(60, 255, 255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Object[] output = contours(findEdges(input), input); //destructive


        /*
        int row_s = 50;
        int row_e = 100;
        int col_s = 50;
        int col_e = 100;
        double total_color = 0;
        for (int r = row_s;row_e > r;r++ ) {
            for (int c = col_s;col_e > c;c++ ) {
                total_color += range.get(5,5)[0];

            }
        }
        currentTotalColor = total_color;

         */
        //Cb.submat(new Rect(region1_pointA, region1_pointB)); this is the region, can use submatrix to figure out the range





        return input;

    }
    public Mat inRange(Mat mat, int h1, int s1,int v1, int h2, int s2, int v2) {
        Scalar clr1 = new Scalar(h1,s1,v1);
        Scalar clr2 = new Scalar(h2,s2,v2);
        Mat res = new Mat();
        Core.inRange(mat, clr1, clr2, res ); //this makes all the true values black, and all the false values white, desecrtuctive on mat
        return res;
        //example: https://stackoverflow.com/questions/36693348/java-opencv-core-inrange-input-parameters
    }
    public Mat findEdges(Mat image) {
        Mat edges = new Mat();
        Imgproc.Canny(image, edges, 100, 300); //destructive
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






}
