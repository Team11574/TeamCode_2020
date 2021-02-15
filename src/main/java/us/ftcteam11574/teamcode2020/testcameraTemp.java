package us.ftcteam11574.teamcode2020;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.ArrayList;

import us.ftcteam11574.teamcode2020.CameraClass;
import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

@TeleOp(name="TMP VISION", group="Iterative Opmode")
public class testcameraTemp extends LinearOpMode {

    int width = 320; // height2  and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvWebcam phoneCam; //more complex form



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);






        // Initialize the back-facing camera
        telemetry.addData("1","");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        telemetry.addData("2","");
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);

        // Connect to the camera
        telemetry.addData("3","");

        //next line is non-blocking!
        detector.hue_low= 15; //change some details to make it a little bit more restricted
        detector.hue_high = 25;
        detector.boxDraw = true;
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }
        });


        //phoneCam.openCameraDevice(); //thihs is syncrononous, which his not great, replace with the async version later which has a call back function
        telemetry.addData("4","");

        // Remember to change the camera rotation!!
        //phoneCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT); //rotation of camera, change if needed
        telemetry.addData("5","");
        // phoneCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED); //possible speed up
        //OpenCvInternalCamera.BufferMethod w= OpenCvInternalCamera.BufferMethod.DOUBLE;
        //...
        telemetry.addData("INIT CAMERA","");


        telemetry.update();

        telemetry.setMsTransmissionInterval(20); //potentiall useful if need fast telemetry
        sleep(500);

        //interesting capabilities
        //phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();

        double v =detector.currentTotalColor;

        telemetry.addData("Test new", v);
        telemetry.update();
        waitForStart();
        int k = 0;
        while(opModeIsActive())
        {
            // Delay to not waste too many CPU cycles


            //use the wideSize to determine how far to move foward. Also, this shoudl have some keeping of the previous positions, so it doesn't just
            //randomly move. I.e. its able to be intelligent.







            telemetry.addData("Update!", k);

            telemetry.update();
        }






    }


}