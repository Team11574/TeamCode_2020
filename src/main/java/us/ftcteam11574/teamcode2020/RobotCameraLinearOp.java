package us.ftcteam11574.teamcode2020;

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

import us.ftcteam11574.teamcode2020.CameraClass;
import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

@TeleOp(name="Test Camera simple2 (THIS)", group="Iterative Opmode")
public class RobotCameraLinearOp extends LinearOpMode {

    int width = 320; // heigh and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvWebcam phoneCam; //more complex form

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake,  Flywheel;
    private Servo Kick;
    private CRServo LeftIn, RightIn;
    private Double power;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LeftIn = hardwareMap.get(CRServo.class, "LeftIn");
        RightIn = hardwareMap.get(CRServo.class, "RightIn");
        Kick = hardwareMap.get(Servo.class, "Kick");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");

        //Set initial position
        Pose2d startPose = new Pose2d(-54, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

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
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT); //rotation of camera, change if needed
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
        sleep(500);
        //interesting capabilities
        //phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();

        double v =detector.currentTotalColor;
        telemetry.addData("Test new", v);
        telemetry.update();

        waitForStart();
        int k = 0;
        while (opModeIsActive())
        {
            // Delay to not waste too many CPU cycles
            sleep(20);

            k++;
            v =detector.returnVal();
            telemetry.addData("Yellow range 1", detector.returnYellows()[0].val[0] + " " + detector.returnYellows()[0].val[1] +" "+detector.returnYellows()[0].val[2]);
            telemetry.addData("Yellow Range 2", detector.returnYellows()[1].val[0] + " " + detector.returnYellows()[1].val[1] +" "+detector.returnYellows()[1].val[2]);
            telemetry.addData("Update!", k);

            telemetry.update();
        }
        //telemtry stuff

        // more robot logic...

        // run until the end of the match (driver presses STOP)

        // Setup a variable for each drive wheel to save power level for telemetry
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-54, 12, Math.toRadians(-90)))
                .splineTo(new Vector2d(-54, 32), Math.toRadians(180))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(10)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(10)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(36)
                .build();


        //(900-500)/2000 = 0.2
        //(2100 - 500)/2000 = 0.8
        Flywheel.setPower(-0.8);
        Kick.setPosition(0.2);
        Kick.setPosition(.45);
        drive.followTrajectory(traj1);
        Kick.setPosition(0.2);
        Kick.setPosition(.45);
        drive.followTrajectory(traj2);
        Kick.setPosition(0.2);
        Kick.setPosition(.45);
        drive.followTrajectory(traj3);
        Kick.setPosition(0.2);
        Kick.setPosition(.45);
        drive.followTrajectory(traj4);
        sleep(1000);
        Flywheel.setPower(0.0);
        drive.followTrajectory(traj5);
    }

}