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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.security.CryptoPrimitive;
import java.security.DomainCombiner;
import java.util.ArrayList;

import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp23", group="Iterative Opmode")
public class TeleOp23 extends OpMode
{
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

    private double FlywheelPower;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean intakeOn = false;
    boolean shot = false;
    double mult = 1;

    double timeSinceShot = 0;

    private int sleepTime = 400;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        //Flywheel setting
        Flywheel.setPower(gamepad2.right_stick_y);
        telemetry.addData("power",gamepad2.right_stick_y);
        Wobble.setPower(gamepad2.left_stick_y*.4); //initialize to the correct position.


        //Mecanum drive

        FRDrive.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
        BRDrive.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);
        FLDrive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        BLDrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);




        //This gives us the option to drive without having to use the tank drive style
        //it seperates the rotation and motion into two seperate joysticks, which can be easier on the driver
        //
        /*
        double[] powers = motorPower.calcMotorsMax(-gamepad1.right_stick_x,gamepad1.right_stick_y, -gamepad1.left_stick_y);

        double motorPow = Math.max(Math.abs(gamepad1.right_stick_x)+Math.abs(gamepad1.right_stick_y) ,Math.abs(gamepad1.left_stick_y));
        FLDrive.setPower(powers[0] * motorPow);
        BLDrive.setPower(powers[1]* motorPow);
        BRDrive.setPower(powers[2]* motorPow);
        FRDrive.setPower(powers[3]* motorPow);
        telemetry.addData("Powers",""+ powers[0] + "," +powers[1] + "," + powers[2]+"," + powers[3]);

         */


        

        //Calc for limiting REV hub's output: (PWM - 500)/2000 for HSR-1425CR Servo
        //(1200-500)/2000 = 0.35
        //(1800 - 500)/2000 = 0.65
        telemetry.addData("A on", intakeOn);
        if(gamepad1.b && !bPressed) {
            if(mult == 0.8) {
                mult = 1;
            }
            else {
                mult = 0.8;
            }
            bPressed = true;
        }
        if(!gamepad1.b) {
            bPressed = false;
        }
        if(!aPressed && gamepad2.y) {
            aPressed = true;
            intakeOn = !intakeOn;
        }
        if(!gamepad2.y) {
            aPressed = false;
        }

        if(intakeOn){
            Roller.setPower(0.65);
            Intake.setPower(-0.8);
            Stationary.setPower(0.8);
        }
        else {
            Roller.setPower(0.0 );
            Intake.setPower(gamepad2.dpad_down?0:0);
            Stationary.setPower(gamepad2.dpad_down?-.42134:0);

        }
        //&& false is temporary.
        if(false && gamepad1.x && runtime.time()-timeSinceShot > 1) { //if you have held the button for more than .5 seconds, then continue with flywheel power
            //if you've continued to hold the button, then only move the Kick into a different position
            timeSinceShot = runtime.time();
        }
        else if(gamepad2.x){
            //shootRing();
            Kick.setPosition(0.5);
            shot = true;
        }
        else {
            Kick.setPosition(0.8);
            shot = false;
            timeSinceShot = runtime.time();
        }


        if(gamepad2.a){
            Gate.setPosition(0.09); //these don't move it enough.
        }
        if(gamepad2.b){
            Gate.setPosition(.89);
            //telemetry.addData("range", gamepad2.right_stick_x);
        }
        if(gamepad1.a){
            Drop.setPower(0.65);
            sleep(100);
            Drop.setPower(0.0);
        }



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    public void shootRing() { //this will trigger after some button has been pressed--it is blocking!

        //Flywheel.setPower(-1); //this coudl be fine tuned to the correct power
        //sleep(1200); //sleeps for some specific amount of time
        Kick.setPosition(0.8);
        sleep(500);
        Kick.setPosition(0.5);
        sleep(500);

        //maybe holding the button shoots more rings
    }


}
