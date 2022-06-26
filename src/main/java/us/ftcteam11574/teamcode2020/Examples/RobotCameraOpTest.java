package us.ftcteam11574.teamcode2020.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import us.ftcteam11574.teamcode2020.CameraClass;

/*

    This op mode can be helpful for testing out the camera, it iwll just return the color in front of it

 */

@TeleOp(name="Test Camera simple", group="Iterative Opmode")
public class RobotCameraOpTest extends LinearOpMode {


    int width = 320; // heigh and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvInternalCamera phoneCam; //more complex form

    @Override
    public void runOpMode() {
         // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        phoneCam.openCameraDevice();

        phoneCam.setPipeline(detector);
        // Remember to change the camera rotation!!
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT); //rotation of camera, change if needed
        //OpenCvInternalCamera.BufferMethod w= OpenCvInternalCamera.BufferMethod.DOUBLE;
        //...
        telemetry.addData("INIT CAMERA","");
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
        telemetry.update();

        telemetry.setMsTransmissionInterval(20); //potentiall useful if need fast telemetry
        sleep(1500);
        //interesting capabilities
        phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();



        double v =detector.currentTotalColor;
        telemetry.addData("Total color", v);
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // Delay to not waste too many CPU cycles
            sleep(20);


            v =detector.currentTotalColor;
            telemetry.addData("Total color", v);

            telemetry.update();
        }
        //telemtry stuff

        // more robot logic...
    }

}