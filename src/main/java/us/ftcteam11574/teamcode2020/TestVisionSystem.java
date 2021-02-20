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

@TeleOp(name="Test Vision", group="Iterative Opmode")
public class TestVisionSystem extends LinearOpMode {

    int width = 320; // height2  and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvWebcam phoneCam; //more complex form

    double wideSizeGoingFor = -1;
    boolean movingToward = false;
    boolean turned = false;

    ArrayList<double[]> memory = new ArrayList<double[]>(10); //the memory this robot contains,
    //stores some data between movements, to allow it to have some memory of where the block was previously.
    //coudl use this to compute if the ring is moving, for example, or if the robot is i simply moving.

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake,  Flywheel;
    private Servo Kick;
    private DcMotor Stationary;
    private Double power;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Kick = hardwareMap.get(Servo.class, "Kick");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        Stationary = hardwareMap.get(DcMotor.class, "Stationary");

        for (int i = 0; 10 > i; i++) {
            memory.add(new double[]{-1,-1,-1,-1}); //adds some blank data, will be replaced later as more data comes in
            //could form this like: <recognized item>, <center>, <distance away x>, <ring moving?>

        }






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
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        waitForStart();
        int k = 0;
        while(opModeIsActive())
        {

            drive.update();
            Pose2d myPose = drive.getPoseEstimate();

            //pose information
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());


            double distance = ( (detector.resLargeBox[0].x +detector.resLargeBox[1].x)/2.0  - width/2); //signed distance //change width/2 to something that accounts for non-central camera
            if(movingToward) {

                /*
                for (int i = 0; 30 > i; i++) {
                    sleep(20);
                    double[] powers = motorPower.calcMotorsFull(0, 0, -1);
                    drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

                }

                 */
                double[] powers = motorPower.calcMotorsFull(0, .8, 0);
                drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                sleep(10);
                //maybe move back a little bit to account for the chagne in the camera position
                /*
                if (distance> 0 ) {
                    double[] powers = motorPower.calcMotorsFull(0, -.8, 0);
                    drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

                    sleep((int)distance * 2); //this should be fine tuned
                }
                else {
                    double[] powers = motorPower.calcMotorsFull(0, .8, 0);
                    drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

                    sleep((int) -distance * 2); //this should be fine tuned
                }

                 */

                drive.turn(Math.toRadians(-90) );
                Intake.setPower(-0.8);
                Stationary.setPower(0.8);
                powers = motorPower.calcMotorsFull(0, 1, 0);
                drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                //should turn the intake on here
                sleep((int) (wideSizeGoingFor * 40) ); //move forward based on how far it is away, this is most likely somewhat wrong.

                powers = motorPower.calcMotorsFull(0, -1, 0);
                drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                sleep((int) (wideSizeGoingFor * 40) );

                drive.turn(Math.toRadians(90) );
                drive.setMotorPowers(0,0,0,0);

                movingToward = false; //could redo the motions to be used with the spline stuff




            }
            else {
                Intake.setPower(0);
                Stationary.setPower(0);
                sleep(5);
                double wideSize = Math.abs(detector.resLargeBox[0].x - detector.resLargeBox[1].x);
                telemetry.addData("Wide", wideSize);
                int closeCutoff = 10;

                if (Math.abs(distance) < closeCutoff) {
                    //then, should brelifly move the correct distance

                    movingToward = true;
                    wideSizeGoingFor = wideSize;
                /*
                if(wideSize < 70) {
                    double[] powers = motorPower.calcMotorsFull(.8, 0, 0);
                    drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                }
                else {
                    drive.setMotorPowers(0,0,0,0);
                }

                 */
                } else {
                    movingToward = false;

                    if ((detector.resLargeBox[0].x + detector.resLargeBox[1].x) / 2.0 > width / 2 && wideSize > closeCutoff) {
                        double[] powers = motorPower.calcMotorsFull(0, -1, 0);
                        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                        telemetry.addData("right", "");
                    } else {
                        if ((detector.resLargeBox[0].x + detector.resLargeBox[1].x) / 2.0 < width / 2 && wideSize > closeCutoff) {
                            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
                            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
                            telemetry.addData("left", "");
                        } else {
                            drive.setMotorPowers(0, 0, 0, 0);
                        }

                    }
                }

                //use the wideSize to determine how far to move foward. Also, this shoudl have some keeping of the previous positions, so it doesn't just
                //randomly move. I.e. its able to be intelligent.

            }





            telemetry.addData("Update!", k);

            telemetry.update();

        }






    }


}