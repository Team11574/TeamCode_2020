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
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
    private VoltageSensor batteryVoltageSensor;
    //Wobble encoder stuff

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone)

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Access battery voltage
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Gate = hardwareMap.get(Servo.class, "Gate");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        Kick = hardwareMap.get(Servo.class, "Kick");
        DcMotor Wobble = hardwareMap.get(DcMotor.class, "Wobble");
        //Wobble run using encoder
        Wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //use the encoders to keep the speed accurate

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
        sleep(1000);
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

        //interesting capabilities
        //phoneCam.setFlashlightEnabled(true);
        //phoneCam.setZoom();

        double v =detector.currentTotalColor;


        telemetry.update();
        int id_pos = maxId(detector.ringsReturn());
        while(!isStarted()) {
            id_pos = maxId(detector.ringsReturn());
            telemetry.addData("Camera position guess",id_pos);
            telemetry.update();
            sleep(250); //dont read too fast, otherwise the input will be a null array. This is much below the fps of 30
        }


        waitForStart(); //need to do something while waiting


        //Store the position indicated by ring stack
        //id_pos = maxId(detector.ringsReturn());
        telemetry.addData("position",id_pos);
        telemetry.update();

        //Set initial position
        Pose2d startPose = new Pose2d(-63.75, 15.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1, powershot, wobbleAdjust, highGoals, launchLine;
        motorPower m = new motorPower();


        double angleAdjust = 0;
        double xAdjust = 0;

        if(id_pos == 1) { //1 rings

            traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(10.75, 25), Math.toRadians(0))
                    .splineTo(new Vector2d(35.75, 21),Math.toRadians(90))
                    .build();
            angleAdjust = 7; //test whether changing this from 3 to 7 was helpful
            xAdjust = -2;
        }
        else if(id_pos == 0) { //0 rings
            traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(10.75, 0), Math.toRadians(90))
                    .build();
            angleAdjust = -4; //was 2
            xAdjust = -3;
        }
        else if(id_pos == 2) { //4 rings
            traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(10.75, 25), Math.toRadians(0))
                    .splineTo(new Vector2d(60.75, 0),Math.toRadians(90))
                    .build();
            angleAdjust = 7;
            xAdjust = -2;
        }
        else { //4 rings
            traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(10.75, 25), Math.toRadians(0))
                    .splineTo(new Vector2d(55.75, 5),Math.toRadians(90))
                    .build();
            angleAdjust = 7;
            xAdjust = -2;
        }


        wobbleAdjust = drive.trajectoryBuilder(traj1.end())
                .back(-10)
                .build();

        if(id_pos == 1) {
            powershot = drive.trajectoryBuilder(wobbleAdjust.end())
                    //.splineTo(new Vector2d(0, wobbleAdjust.end().component2()), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-40, 2,Math.toRadians(151)))
                    .build();
        }
        else {
            powershot = drive.trajectoryBuilder(wobbleAdjust.end())
                    .splineTo(new Vector2d(-5 + xAdjust, 2), Math.toRadians(156 + angleAdjust))
                    .build();
        }
        launchLine = drive.trajectoryBuilder(powershot.end())
                .back(6-xAdjust)//check whether xadjust works
                .build();

        drive.followTrajectory(traj1);
        //This wobble works great though
        Wobble.setPower(.7 * (13.1 / batteryVoltageSensor.getVoltage()));
        sleep(500);
        Gate.setPosition(0.89);
        sleep(200);
        drive.followTrajectory(wobbleAdjust);
        Wobble.setPower(-1);
        sleep(1200);
        Wobble.setPower(0);
        //Position for shooting
        drive.followTrajectory(powershot);
        //Flywheel
        telemetry.update();

        //Alter flywheel power based on voltage to ensure consistent shots
        double flywheelPower = -1.0 * (13.1 / batteryVoltageSensor.getVoltage());

        //-----Shot 1-----
        Kick.setPosition(0.8);
        Flywheel.setPower(flywheelPower);
        sleep((long) (2400) );
        Kick.setPosition(0.4);
        sleep(100);
        Kick.setPosition(0.8);
        //-----Shot 2-----

        sleep(1000);
        Kick.setPosition(0.4);
        sleep(100);
        Kick.setPosition(0.8);
        //-----Shot 3-----
        sleep(1000);
        Kick.setPosition(0.4);
        sleep(100);
        Kick.setPosition(0.8);
        //-----Shot 4 extra-----
        sleep(1150);
        Kick.setPosition(0.4);
        sleep(100);
        Kick.setPosition(0.8);
        //Turn off flywheel
        Flywheel.setPower(0.0);


        //Launch line
        telemetry.addData("Runtime",getRuntime());
        telemetry.update();
        drive.followTrajectory(launchLine);


        //drive.turn(Math.toRadians(90));

        //Adjust position to which we spline depending on desired target.
        /*
        double yadjust = 0;
        double xadjust = 0;
        if(id_pos == 0){
            yadjust = 15.75-23;
            xadjust = traj1.end().component2(); //keep the same x component
        }else if(id_pos == 2){
            yadjust = 15.75-23;
            xadjust = traj1.end().component2()+10;
        } else if(id_pos == 1) {
            yadjust = 38.75;
            xadjust = traj1.end().component2()+10;
        }


        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d((xadjust),(yadjust), Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj2);

         */


        /* Wobble isn't responding to encoders atm.
        Wobble.setTargetPosition(1120/2);
        Wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Servo calc example: (900-500)/2000 = 0.2 (2100 - 500)/2000 = 0.8
        Gate.setPosition(0.9);
        Wobble.setTargetPosition(0);
        Wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */



        //Last trajectory for powershots. This needs to be tuned!
            //For me, the spline is acting very odd. For now, I'll use a turn and backwards movement
        /*
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(40, 10), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj3);

        drive.turn(Math.toRadians(90));
        if(id_pos == 1) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(traj2.end().component1(), traj2.end().component2(), traj2.end().component3() + Math.toRadians(90)))
                    .splineTo(new Vector2d(-40, 15), Math.toRadians(180))
                    .build());
        }
        if(id_pos != 1) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(traj2.end().component1(), traj2.end().component2(), traj2.end().component3() + Math.toRadians(90)))
                    .splineTo(new Vector2d(-10, -20), Math.toRadians(180)) //this needs some testing to ensure the y component is  correct
                    .build());
        }
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
