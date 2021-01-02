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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;


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

@TeleOp(name="Autonomous", group="Linear Opmode")
public class Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BRDrive = null;
    DrivingPoint target1;
    ArrayList<DrivingPoint> queue = new ArrayList<DrivingPoint>();

    //Last encoder positions
    private int lastFL = 0;
    private int lastFR = 0;
    private int lastBL = 0;
    private int lastBR = 0;
    //We need to measure the field and find the actual initial X and Y
    private double lastX = 0;
    private double lastY = 0;

    //Wheel displacement per encoder count
    private double displacementPerCount = (10.16*Math.PI)/1120;

    // Our sensors, motors, and other devices go here, along with other long term state
    BNO055IMU imu;
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    //PID controllers
    PIDController xController, yController, rotController;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Drivetrain hardwaremap
        FLDrive  = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive  = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive  = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive  = hardwareMap.get(DcMotor.class, "BRDrive");
        //IMU hardwaremap and initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        //Define target points
        target1 = new DrivingPoint(2, 2);

        //Reset encoders
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Compute robot's position
            //Compute change in encoder positions
            double deltaFL = FLDrive.getCurrentPosition() - lastFL;
            double deltaFR = FRDrive.getCurrentPosition() - lastFR;
            double deltaBL = BLDrive.getCurrentPosition() - lastBL;
            double deltaBR = BRDrive.getCurrentPosition() - lastBR;
            //Compute displacements for each wheel
            double displFL = deltaFL * displacementPerCount;
            double displFR = deltaFR * displacementPerCount;
            double displBL = deltaBL * displacementPerCount;
            double displBR = deltaBR * displacementPerCount;
            //Compute the average displacement in order to untangle rotation from displacement
            double displAvg = (displFL + displFR + displBL + displBR) / 4.0;
            //Compute the component of the wheel displacements that yield robot displacement
            double devFL = displFL - displAvg;
            double devFR = displFR - displAvg;
            double devBL = displBL - displAvg;
            double devBR = displBR - displAvg;
            //Compute the displacement of the holonomic drive, in robot reference frame
            double delt_Xr = (devFL + devFR - devBL - devBR) / (2* Math.sqrt(2));
            double delt_Yr = (devFL - devFR - devBL + devBR) / (2* Math.sqrt(2));
            //Move this holonomic displacement from robot to field frame of reference
            double robotTheta = angles.firstAngle;
            double sinTheta = Math.sin(robotTheta);
            double cosTheta = Math.cos(robotTheta);
            double delt_Xf = delt_Xr * cosTheta - delt_Yr * sinTheta;
            double delt_Yf = delt_Yr * cosTheta + delt_Xr * sinTheta;
            //Update the position
            double X = lastX + delt_Xf;
            double Y = lastY + delt_Yf;
            double Theta = robotTheta;
            lastFL = FLDrive.getCurrentPosition();
            lastFR = FRDrive.getCurrentPosition();
            lastBL = BLDrive.getCurrentPosition();
            lastBR = BRDrive.getCurrentPosition();

            //Check the next drivingpoint in the queue and use PID to get to the next point.
            //Setting up the PID controllers
            //NONE OF THE VALS ARE CALIBRATED ATM
            xController = new PIDController(.003, .00003, 0);
            yController = new PIDController(.003, .00003, 0);
            rotController = new PIDController(.003, .00003, 0);

            if (queue.size() != 0){
                xController.setSetpoint(queue.get(0).xval());
                yController.setSetpoint(queue.get(0).yval());
                rotController.setSetpoint(Math.atan2(X, Y) - Theta);

                double xPower = xController.performPID();
                double yPower = yController.performPID();
                double rotPower = rotController.performPID();
                //Not finished

                
            }else{

            }
        }
    }
}
