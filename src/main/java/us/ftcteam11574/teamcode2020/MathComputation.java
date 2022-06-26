package us.ftcteam11574.teamcode2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="Calibrate Launcher", group="Iterative Opmode")
//TODO:
    //besides pathing--what should I program?
    //autonomous coudl use some work, coudl program a processing thing to simulate the program, coudl use that for pathing
    //If i did this as a reinforcement learning kind of thing, woudl need to make sure I factor in errors and unrealiablity stuff
    //COudl at least train it to do well in autonomous, sicne then it has all the information about its own locations and stuff
    //
//computes the math from a specific distance--calibrates information.
//pathign stuff without using mechanum
//program a search alorgithm to find paths in processing
//plan, takes power form encoders, and predicts distance. Chgnes power bsad on whats needed
//work on EN
//I could expect that the powerscales in an odd way, so maybe graph different poitns to find how the power ramsp with
//distance
//shoudl change pwoer of motors based on different speed, try to run at specific speed that is neede
public class MathComputation extends OpMode {
    public static DcMotor input = null; //input power to the launcher
    public static double angle = Math.PI/4; //in radians
    public static double height_change = 1.1; //in meters
    boolean shooting = true;
    double add_distance = 0;
    double value_power = 0;
    boolean click = false;
    double pos_p = 0;
    double time_p= 0;
    public static double g =9.8;



    public static double angleToGoal(double currentX, double currentY) {
        //TODO
        //should return arctan()
        //could consider the goal to be (0,0)?
        return Math.atan2(currentY, currentX);
    }
    public static double predictDistance(double predictedPower) {
        double dist = predictedPower * Math.cos(angle) * 2*predictedPower*Math.sin(angle);
        dist /= g;
        return dist;

    }
    public static double idealPower(double l) {
        double r = Math.sqrt((-g*l)/((height_change-l*Math.tan(angle))*(2* Math.pow(Math.cos(angle),2))) );
        return r;
    }
    public static double convertToPower(double speed) {
        //TODO
        return speed;
    }
    public static double powerDiff(double goalSpeed) {
        //return currentSpeed-goalSpeed;
        return 0;

    }
    @Override
    public void init() {
        input  = hardwareMap.get(DcMotor.class, "launch");
        input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        time_p = getRuntime();
    }
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


    }
    @Override
    public void loop() {

        telemetry.addData("Mode",shooting);
        if(gamepad1.a){
            shooting = !shooting;
        }
        if (shooting) {

            input.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Power",-gamepad1.right_stick_y);
        }
        else {
            //input distance
            //add_distance
            double speed = input.getCurrentPosition()-pos_p;
            speed /= (getRuntime()-time_p);
            telemetry.addData("Diff","" + (getRuntime()-time_p) );
            telemetry.addData("Diff2","" + (input.getCurrentPosition()) );

            if(gamepad1.b) {
                if(click) {
                    value_power += .01;
                }
                click= false;
            }
            else {
                click = true;
            }
            if(gamepad1.x) {
                input.setPower(value_power);
            }
            //input.setPower(value_power);


            telemetry.addData("Speed",speed);
            telemetry.addData("Power2",value_power);
            pos_p = input.getCurrentPosition();
            time_p = getRuntime();



        }


    }

}
