package us.ftcteam11574.teamcode2020.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import us.ftcteam11574.teamcode2020.Skyblock;

/*
    This file shows an example of using the camera
 */

@Disabled
@TeleOp(name="Camera", group="Iterative Opmode")
public class Camera extends LinearOpMode {
    public static WebcamName webCam;
    public static VuforiaLocalizer vuforia;
    boolean rgb_format_worked;

    @Override
    public void runOpMode() throws InterruptedException {




        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);




        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vu_parameters.vuforiaLicenseKey = "AV29AFb/////AAABma3tuKm8DE2/tKJA0LIvwcIWOzMsiVbx8yLAiSRl1l98p84lwbzzJMkqsJw7ySFusaR6sYtQoSN9rzPIjUVqJ/uLkqv/V0rllY9LtZS0bnUfiyYarG+ZIDk587QhB/+BdT2EMo7w7+dHPO3Y9YOoFMZom016W6kYU+Tc7/OaN0AMXb6zGal02KRH3h913F+84o7J48sKXz0whgL1TSbfFQvYYyzijQlqzsmcvee4e3AI3L30L9AM1+COMhKcsIuYjpuUl1/oELl6XSCC7Q3UVnrKnah1WQb2C8m1KdsGgPbPp42rFC4ArXydJI193CEEENY/fyHvxxh8/aEb4fxxmybXkPk93BVpPZL6co8hFpSF";


        vu_parameters.cameraName = webCam;

        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);
        vuforia.enableConvertFrameToBitmap();

       // vuforia.getCameraCalibration();


        rgb_format_worked = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB888)[0];

        telemetry.addData("test","test");
        vuforia.setFrameQueueCapacity(3);

        //GGGRRRRR BBBBBGGG
        //



        for (int time = 0; 10 > time; time++) {

            Image img = readCamera();

            Object[] res = readImageRGB565(img);
            //gets to here succesfully
            //telemetry.addData("Size width", (int) res[1]);
            //telemetry.addData("Size height", (int) res[2]);
            //telemetry.update();
            long time_start = System.currentTimeMillis();
            Skyblock.readImage(((int[][]) (res[0])), (int) res[1], (int) res[2]);
            //Skyblock.score(Skyblock.pixels); //might just be running too slow, so it crashes

            double[][] regions = Skyblock.bestRegion(Skyblock.score(Skyblock.pixels));

            double max = 0;
            int max_id = 0;

            for (int i = 0; regions.length > i; i++) {
                if (regions[i][0] > max) {
                    max_id = i;
                    max = regions[i][0];

                }
            }

            double[] center = new double[]{regions[max_id][2], regions[max_id][1]};
            telemetry.addData("Position", "x" + center[0] + "y" + center[1]);
            telemetry.addData("Time",System.currentTimeMillis()-time_start);

            //how long it took to recognize the image


            telemetry.update();
        }






    }
    public Object[] readImageRGB565(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();

        java.nio.ByteBuffer pixels = image.getPixels();

        //System.out.println("the byte order is" + pixels.order());
        int[][] rgb = new int[width*height][3];
        //telemetry.addData("type",pixels.order());
        //ByteOrder.
        //ASSUMING THAT BYTEORDER IS BIG ENDIAN
        int i = 0;
        while(pixels.hasRemaining()) {

            int read1 = (((Byte)pixels.get()).intValue() << 8) | ((Byte)pixels.get()).intValue();
            //int read2 = ((Byte)pixels.get()).intValue();



            int red_read = ( (read1 & 0b11111000_00000000) >> 11) << 3 ; //read the first 5 bits
            int green_read = ( (read1 & 0b00000111_11100000) >> 5) << 2;
            int blue_read = ( (read1 & 0b00000000_00011111) >> 0 ) << 3;




            rgb[i][0] =   red_read;
            rgb[i][1] =   green_read;
            rgb[i++][2] =   blue_read;



        }
        return new Object[]{rgb,width,height};


    }

    public static Object[] readImageRGB888(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        java.nio.ByteBuffer pixels = image.getPixels();
        int i = 0;
        System.out.println("the byte order is" + pixels.order());
        int[][] rgb = new int[width*height][3];
        while(pixels.hasRemaining()) {
            int red = ((Byte)pixels.get()).intValue();
            int green = ((Byte)pixels.get()).intValue();
            int blue = ((Byte)pixels.get()).intValue();
            rgb[i][0]   = red;
            rgb[i][1]   = green;
            rgb[i++][2] = blue;
            //hopefully, this convert correctly, this will need some testing

            /*
            convertByte(red,pixels.order());
            */

            //WE ARE ASSUMING LENGTH OF PIXELS % 3 == 0 !!!
            //MIGHT CRASH!
            //
        }
        return new Object[]{rgb,width,height};

        //pixels are in the form:
        //RRRRRRRR GGGGGGGG BBBBBBB
    }
    public Image readCamera() {
        VuforiaLocalizer.CloseableFrame frame = null;
        Image rgb = null;
        try {
            frame = vuforia.getFrameQueue().take();
            telemetry.addData("images",frame.getNumImages());
            long numImages = frame.getNumImages();
            for (int i = 0; numImages > i; i++) {
                telemetry.addData("rgb format",frame.getImage(i).getFormat());

                //right now only reading the color greyscale

                if(frame.getImage(i).getFormat()==PIXEL_FORMAT.RGB888) {

                    rgb = frame.getImage(i);
                    //should break after this?
               }
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return rgb;
    }

    //webCam = hardwareMap.get(WebcamName.class, "Webcam 1");

}
