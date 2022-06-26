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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import us.ftcteam11574.teamcode2020.drive.SampleMecanumDrive;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="LastMinute", group="Pushbot")
public class LastMinute extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor FLDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor Intake = null;
    private DcMotor Flywheel = null;
    private CRServo LeftIn = null;
    private CRServo RightIn = null;
    private Servo Kick = null;
    private Servo Gripper = null;
    private CRServo Extender = null;
    private DcMotor Wobble = null;
    private double FlywheelPower;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        FLDrive  = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive  = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive  = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive  = hardwareMap.get(DcMotor.class, "BRDrive");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LeftIn = hardwareMap.get(CRServo.class, "LeftIn");
        RightIn = hardwareMap.get(CRServo.class, "RightIn");
        Kick = hardwareMap.get(Servo.class, "Kick");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        Extender = hardwareMap.get(CRServo.class, "Extender");
        Wobble = hardwareMap.get(DcMotor.class, "Wobble");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        FLDrive.setPower(1.0);
        FRDrive.setPower(1.0);
        BLDrive.setPower(1.0);
        BRDrive.setPower(1.0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
