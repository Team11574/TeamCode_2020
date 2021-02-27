package us.ftcteam11574.teamcode2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

@TeleOp(name="Final Autonomous", group="Linear Opmode")
public class FinalAutonomous extends LinearOpMode {

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
    //Wobble encoder stuff

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
        Kick = hardwareMap.get(Servo.class, "Kick");
        DcMotor Wobble = hardwareMap.get(DcMotor.class, "Wobble");
        //Wobble run using encoder
        Wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize webcam
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

        //Store the position indicated by ring stack
        int id_pos = maxId(detector.ringsReturn());
        telemetry.addData("position",id_pos);
        telemetry.update();

        //Set initial position
        Pose2d startPose = new Pose2d(-63.75, 15.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        //Establish expected position after 90 deg turn
        Pose2d turnPose = new Pose2d(-60.75, 15.75, Math.toRadians(0));
        Trajectory traj1;
        Trajectory traj2;
        motorPower m = new motorPower();

        //m.moveDirMaxRamp(0,1,0,1000,drive);

        if(id_pos == 1) { //1 rings
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(110)//good distance
                    .build();
        }
        else if(id_pos == 0) { //0 rings
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(70)
                    .build();
        }
        else if(id_pos == 2) { //4 rings
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(120)
                    .build();
        }
        else { //4 rings
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(120)
                    .build();
        }
        //
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj1);
        //drive.turn(Math.toRadians(90));

        //Adjust position to which we spline depending on desired target.
        int yadjust = 0;
        int xadjust = 0;
        if(id_pos == 0){
            yadjust = 50;
            xadjust = -15;
        }else if(id_pos == 2){
            yadjust = 70;
            xadjust = 30;
        } else if(id_pos == 1) {
            yadjust = 10;
            xadjust = 0;

        }
        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d((49.25 + xadjust),(30-yadjust), Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj2);
        /* Wobble isn't responding to encoders atm.
        Wobble.setTargetPosition(1120/2);
        Wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Servo calc example: (900-500)/2000 = 0.2 (2100 - 500)/2000 = 0.8
        Gate.setPosition(0.9);
        Wobble.setTargetPosition(0);
        Wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        //This wobble works great though
        Wobble.setPower(.4);
        sleep(1000);
        Gate.setPosition(0.89);
        sleep(500);
        Wobble.setPower(-0.6);
        sleep(1500);
        Wobble.setPower(0);

        //Last trajectory for powershots. This needs to be tuned!
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(40, -20), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj3);

        /*
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
        sleep(2750);
        if(id_pos == 1) {
            drive.turn(-5);
        }
        drive.setMotorPowers(0,0,0,0);
        Flywheel.setPower(-1); //this coudl be fine tuned to the correct power
        sleep(1600); //sleeps for some specific amount of time
        //Kick.setPosition(0.8);
        Kick.setPosition(0.5);
        sleep(200);
        Kick.setPosition(0.8);


        Flywheel.setPower(-1);
        sleep(800); //extra time to gain power

        sleep(200);
        Kick.setPosition(0.5);
        sleep(200);
        Kick.setPosition(0.8);

        Flywheel.setPower(-1);
        sleep(800); //extra time to gain power

        sleep(200);
        Kick.setPosition(0.5);
        sleep(200);
        Kick.setPosition(0.8);
        Flywheel.setPower(0.0);

        Flywheel.setPower(0.0);

        powers = motorPower.calcMotorsFull(0, -1, 0);
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
        sleep(500);
        drive.setMotorPowers(0,0,0,0);


        //drive.followTrajectory(traj2);
        //shoot the rings during this period
        //drive.followTrajectory(traj2);
        //drive.followTrajectory(traj3);

        /*d
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
