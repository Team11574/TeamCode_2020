package us.ftcteam11574.teamcode2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import us.ftcteam11574.teamcode2020.CameraClass;
import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

import static android.os.SystemClock.sleep;

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
    private Servo Gate;
    private CRServo LeftIn, RightIn;
    private Double power;
    private Servo Kick;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Gate = hardwareMap.get(Servo.class, "Gate");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        DcMotor Wobble = hardwareMap.get(DcMotor.class, "Wobble");
        Kick = hardwareMap.get(Servo.class, "Kick");
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

        //next line is non-blocking!
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

        motorPower.motors = new DcMotor[]{hardwareMap.get(DcMotorEx.class, "FLDrive"), hardwareMap.get(DcMotorEx.class, "BLDrive"), hardwareMap.get(DcMotorEx.class, "BRDrive"), hardwareMap.get(DcMotorEx.class, "FRDrive")};
        telemetry.update();

        telemetry.setMsTransmissionInterval(20); //potentiall useful if need fast telemetry
        sleep(500);
        //interesting capabilities
        //phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();

        double v =detector.currentTotalColor;
        telemetry.addData("Test new", maxId(detector.ringsReturn()) );

        telemetry.update();

        waitForStart();



        //telemtry stuff

        // more robot logic...

        // run until the end of the match (driver presses STOP)

        // Setup a variable for each drive wheel to save power level for telemetry
        int id_pos = maxId(detector.ringsReturn());

        telemetry.addData("position",id_pos);
        telemetry.update();


        Pose2d turnPose = new Pose2d(-54, 12, Math.toRadians(90));
        Trajectory traj1;
        motorPower m = new motorPower();

        //m.moveDirMaxRamp(0,1,0,1000,drive);

        if(id_pos == 1) {
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(120) //need to go further.
                    .build();
        }
        else if(id_pos == 0) {
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(90)
                    .build();
        }
        else if(id_pos == 2) {
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(90)
                    .build();
        }
        else {
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(170)
                    .build();
        }

        Trajectory traj2= drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-48,53),Math.toRadians(-90))
                .build();
        //
        //


        //(900-500)/2000 = 0.2
        //(2100 - 500)/2000 = 0.8

        //Flywheel.setPower(-0.8);
        drive.turn(Math.toRadians(-90));

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        if(id_pos == 0 || id_pos == 2) {
            double[] powers = motorPower.calcMotorsFull(0, -1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            sleep(1500);
            drive.setMotorPowers(0,0,0,0);

        }
        else if(id_pos == 1) {
            double[] powers = motorPower.calcMotorsFull(0, -1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(700);
            drive.setMotorPowers(0,0,0,0);
        }
        Wobble.setPower(.5);
        Gate.setPosition(.1);
        sleep(1000);
        //Gate.setPosition(0); //gate doesn't seem to be working all that well, so were just relying on it falling out.
        sleep(1500);
        Wobble.setPower(-1);
        sleep(500);
        Wobble.setPower(0);
        if(id_pos == 0 || id_pos == 2) {
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(1500);
            drive.setMotorPowers(0,0,0,0);

        }
        else if(id_pos == 1) {
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(700);
            drive.setMotorPowers(0,0,0,0);
        }
        //if id_pos == 1
        if(id_pos == 0) {
            drive.turn(Math.toRadians(90));
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(800);
            drive.setMotorPowers(0, 0, 0, 0);
        }
        if(id_pos == 1) {
            drive.turn(Math.toRadians(90));
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(2000);
            drive.setMotorPowers(0, 0, 0, 0);
        }
        if(id_pos == 2) {
            drive.turn(Math.toRadians(90));
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(3000);
            drive.setMotorPowers(0,0,0,0);
        }
        double[] powers = motorPower.calcMotorsFull(-.8, 0, 0); //strafe somewhat slowly, since it probably won't be consistent
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
        sleep(3000);
        drive.setMotorPowers(0,0,0,0);
        Flywheel.setPower(1); //this coudl be fine tuned to the correct power
        sleep(2400); //sleeps for some specific amount of time
        Kick.setPosition(0.8);
        Kick.setPosition(0.5);
        sleep(1000);
        Kick.setPosition(0.8);
        Kick.setPosition(0.5);
        sleep(1000);
        Flywheel.setPower(0.0);
        powers = motorPower.calcMotorsFull(0, -1, 0);
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
        sleep(500);
        drive.setMotorPowers(0,0,0,0);


        //drive.followTrajectory(traj2);
        //shoot the rings during this period
        //drive.followTrajectory(traj2);
        //drive.followTrajectory(traj3);

        /*
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

         */


    }
    int maxId(int[] res) {
        int max = -1;
        int maxId = 0;
        for(int i = 0; res.length > i; i++) {
            if(res[i] > max) {
                max = res[0];
                maxId = i;
            }
        }
        return maxId;




    }

}
