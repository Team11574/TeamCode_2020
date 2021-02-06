/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package us.ftcteam11574.teamcode2020;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Vector;

import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

import static java.lang.Math.max;
import static java.lang.Math.min;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PIDAutonomous", group="Linear Opmode")
public class PIDAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake,  Flywheel;
    private Servo Kick;
    private CRServo LeftIn, RightIn;
    private Double power;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {

        // Setup a variable for each drive wheel to save power level for telemetry
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-54, 12, Math.toRadians(180)))
                .strafeLeft(20)
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
        //}
    }
}

