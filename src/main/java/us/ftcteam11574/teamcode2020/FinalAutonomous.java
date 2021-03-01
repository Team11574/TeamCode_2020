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
    //Wobble encoder stuff
    //VoltageSensor voltSensor; //If possible, I should try to implement something that keeps motor pwoer as a factor, not exactly sure how though
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

        //Establish expected position after 90 deg turn
        Pose2d turnPose = new Pose2d(-60.75, 15.75, Math.toRadians(0));
        Trajectory traj1;
        Trajectory traj11 = null; //needs to be initlized
        Trajectory traj2;
        motorPower m = new motorPower();

        //probably need a short strafe to avoid the ring


        if(id_pos == 1) { //1 rings

            traj11 = drive.trajectoryBuilder(turnPose)
                    .strafeLeft(20) //hopefully this will avoid the rings.
                    //good distance
                    .build();
            traj1 = drive.trajectoryBuilder(traj11.end())
                    .forward(110) //hopefully this will avoid the rings.
                    //good distance
                    .build();
        }
        else if(id_pos == 0) { //0 rings
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(70)
                    .build();
        }
        else if(id_pos == 2) { //4 rings
            traj11 = drive.trajectoryBuilder(turnPose)
                    .strafeLeft(20) //hopefully this will avoid the rings.
                    //good distance
                    .build();
            traj1 = drive.trajectoryBuilder(traj11.end())
                    .forward(120)
                    .build();
        }
        else { //4 rings
            traj11 = drive.trajectoryBuilder(traj11.end())
                    .strafeLeft(20) //hopefully this will avoid the rings.
                    //good distance
                    .build();
            traj1 = drive.trajectoryBuilder(turnPose)
                    .forward(120)
                    .build();
        }
        //

        drive.turn(Math.toRadians(-90));
        if(id_pos != 0) {
            drive.followTrajectory(traj11);
        }
        drive.followTrajectory(traj1);

        //drive.turn(Math.toRadians(90));

        //Adjust position to which we spline depending on desired target.
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
            //For me, the spline is acting very odd. For now, I'll use a turn and backwards movement
        /*
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(40, 10), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj3);
         */
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




        //we reangle the laucnher so we are facing towards the correct location
        //drive.turn(Math.toRadians(-25) ); //this is the angle to shoot towards the high goal-- needs lots of battery power to make this shot
        //should probably try to set the flywheel power to the
        //then we shoot.\




        telemetry.update();
        //we base off of the 14.00 power. If it falls to much below that amoutn the flywheel will work inconsitantely
        /*
        "


         */
        //-----Shot 1-----
        Flywheel.setPower(-1); //this coudl be fine tuned to the correct power
        sleep((long) (2400) ); //sleeps for some specific amount of time. Should be long enough to achieve the right speed with all motor powers.
        //Kick.setPosition(0.8);
        Kick.setPosition(0.5);
        sleep(150); //wait less time, to hopefully decrease odds of a double or triple shot at once.
        //This means that we ned to set the robot up so that the pusher is very close to the rings
        Kick.setPosition(0.4); //move a little forward to catch the ring
        //-----Shot 1-----

        //-----Hold 1-----
        Flywheel.setPower(-1);
        //drive.turn(Math.toRadians(-12) );
        sleep(300); //we don't need to keep it for tha tlong before the next shot
        Kick.setPosition(0.5);
        sleep(150);
        Kick.setPosition(0.8);
        //-----Hold 1-----

        //-----Shot 2-----
        Flywheel.setPower(-1);
        sleep(600); //
        Kick.setPosition(0.5);
        sleep(100);
        Kick.setPosition(0.75);
        //-----Shot 2-----

        //-----Hold 2-----
        Flywheel.setPower(-1);
        //drive.turn(Math.toRadians(-12) );
        sleep(200); //we don't need to keep it for tha tlong before the next shot

        sleep(150);
        Kick.setPosition(0.8);
        //-----Hold 2-----

        //-----Shot 3-----
        Flywheel.setPower(-1);
        sleep(200);
        Kick.setPosition(0.5);
        sleep(200);
        Kick.setPosition(0.8);
        Flywheel.setPower(0.0);
        //-----Shot 3-----

        Flywheel.setPower(0.0);



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
