package us.ftcteam11574.teamcode2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvWebcam;

import us.ftcteam11574.teamcode2020.CameraClass;
import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

@TeleOp(name="TEST 2", group="Iterative Opmode")
public class TestOpMode extends LinearOpMode {

    int width = 320; // heigh and width of the camera
    int height = 240;

    CameraClass detector = new CameraClass();
    // OpenCvCamera phoneCam; //simpler version
    OpenCvWebcam phoneCam; //more complex form

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake, Flywheel;
    private Servo Gate;
    private CRServo LeftIn, RightIn;
    private Double power;


    @Override
    public void runOpMode() {
        super.waitForStart();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-54, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(0, 12))
                .splineTo(new Vector2d(20, 20), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj1);

    }
}