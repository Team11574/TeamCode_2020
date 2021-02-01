package us.ftcteam11574.teamcode2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import us.ftcteam11574.teamcode2020.CameraClass;

@TeleOp(name="Test Camera simple2 (THIS)", group="Iterative Opmode")
public class RobotCameraLinearOp extends LinearOpMode {


    int width = 320; // heigh and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvWebcam phoneCam; //more complex form

    @Override
    public void runOpMode() {
        // Initialize the back-facing camera
        telemetry.addData("1","");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        telemetry.addData("2","");
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        // Connect to the camera
        telemetry.addData("3","");

        //next line is blocking!
        phoneCam.openCameraDevice(); //thihs is syncrononous, which his not great, replace with the async version later which has a call back function
        telemetry.addData("4","");
        phoneCam.setPipeline(detector);
        // Remember to change the camera rotation!!
        phoneCam.startStreaming(width, height); //rotation of camera, change if needed
        telemetry.addData("5","");
        // phoneCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED); //possible speed up
        //OpenCvInternalCamera.BufferMethod w= OpenCvInternalCamera.BufferMethod.DOUBLE;
        //...
        telemetry.addData("INIT CAMERA","");

        /*
        for (OpenCvInternalCamera.FrameTimingRange r : phoneCam.getFrameTimingRangesSupportedByHardware())
        {
            //could telemetry these out to see what is availbe
            telemetry.addData("CAMERA FPS OPTIONS","max" + r.max + "  min:" + r.min);
            if(r.max == 30 && r.min == 30) //if locked at 30, otherwise, don't change the frameTiming

            {
                phoneCam.setHardwareFrameTimingRange(r);
                break;
            }
        }

         */
        telemetry.update();

        telemetry.setMsTransmissionInterval(20); //potentiall useful if need fast telemetry
        sleep(1500);
        //interesting capabilities
        //phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();



        double v =detector.currentTotalColor;
        telemetry.addData("Total color", v);
        telemetry.update();

        waitForStart();
        int k = 0;
        while (opModeIsActive())
        {
            // Delay to not waste too many CPU cycles
            sleep(20);

            k++;
            v =detector.returnVal();
            telemetry.addData("Total color", detector.a);
            telemetry.addData("Total color2", detector.b);
            telemetry.addData("Total color3", detector.c);
            telemetry.addData("Update!", k);

            telemetry.update();
        }
        //telemtry stuff

        // more robot logic...
    }

}