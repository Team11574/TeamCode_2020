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

package us.ftcteam11574.teamcode2020.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;
import us.ftcteam11574.teamcode2020.motorPower;



/*

    This can be used as an example of just a very simple autonomous mode to get you started

 */

@TeleOp(name="League Last Minute", group="Linear Opmode")
public class LeagueLastMinute extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor Intake = null;
    private DcMotor Flywheel = null;
    private DcMotor Wobble = null;
    private DcMotor Stationary = null;
    private Servo Gate = null;
    private Servo Kick = null;
    private CRServo Roller = null;
    private CRServo Drop = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FLDrive  = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive  = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive  = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive  = hardwareMap.get(DcMotor.class, "BRDrive");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Stationary = hardwareMap.get(DcMotor.class, "Stationary");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        Wobble = hardwareMap.get(DcMotor.class, "Wobble");
        Gate = hardwareMap.get(Servo.class, "Gate");
        Kick = hardwareMap.get(Servo.class, "Kick");
        Roller = hardwareMap.get(CRServo.class, "Roller");
        Drop = hardwareMap.get(CRServo.class, "Drop");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        Stationary.setDirection(DcMotor.Direction.REVERSE);
        Pose2d startPose = new Pose2d(-54, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

            /*
            double[] powers = motorPower.calcMotorsFull(0, 1, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(600);
            powers = motorPower.calcMotorsFull(0, 0, -1);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(307);
            powers = motorPower.calcMotorsFull(0, 0, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            */
            double[] powers = motorPower.calcMotorsFull(0, 0, -1);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(297);
            powers = motorPower.calcMotorsFull(0, 0, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            Flywheel.setPower(-1); //this coudl be fine tuned to the correct power
            sleep(300); //sleeps for some specific amount of time //must be at max battery for this to work well
            //Kick.setPosition(0.8);
            Kick.setPosition(0.5);
            sleep(200);
            Kick.setPosition(0.8); //shot 1


            //powers = motorPower.calcMotorsFull(0, .7, 0);
            //drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);



            sleep(250); //extra time to gain power

            sleep(200);
            //powers = motorPower.calcMotorsFull(0, 0, -.4);
            //drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            Kick.setPosition(0.5);
            sleep(200);
            Kick.setPosition(0.8); //shot 1
            powers = motorPower.calcMotorsFull(0, 0, -1);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            sleep(327);
            powers = motorPower.calcMotorsFull(0, 0, 0);
            drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            Flywheel.setPower(-1);
            sleep(850); //extra time to gain power

            sleep(200);
            Kick.setPosition(0.5);
            sleep(200);
            Kick.setPosition(0.8); //shot 3
            Flywheel.setPower(0.0);


            sleep(500);
            //powers = motorPower.calcMotorsFull(0, 0, 0);
            //drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);






    }
}
