package us.ftcteam11574.teamcode2020;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.lang.Math;
//Configuration looks like this
// \/
// /\
@Disabled
public class motorPower {
    //pretty much everythign static so it can be accessed without instantiating a <Robotics> object
    /*
    public static void main(String[] args) {
        System.out.println(calcv_0(-0.813,.495,1.22,50));
    }
    */


    public static final double eps = .00001; //epsiolon when computing calcv_0
    static double rampUp(double cur_time, double in_time, double low, double hi) {
        //rampUp to hi from hi to low
        //does this in <in_time>
        //pass in cur time to get the current power you should have to ramp up
        double val = low + (hi-low)*(sigma( ( (cur_time*2.2681)/in_time) + p )-fp );
        if (val > hi) {
            return hi;
        }
        if (val < low) {
            return low;
        }
        return val;
    }
    static double p = -1.27856454276107379510935873902298015543947748861974576545d; //approximation of the x value of the absolute min
    static double fp = sigma(p);
    //a little bit is added to this to ensure the direction doesn't change because of rounding errors
    static double sigma(double x) {
        return (x/(1.+Math.pow(Math.E,-x)));
    }
    static double calcv_0(double vx,double vy, double rot) {
        /*
            Calcuates the value of v_0 which will esult in the smallest max(new double[]{v_0,v_1,v_2,v_3} );
        */

        return calcv_0(vx,vy,rot,50); //gets max accuracy at around 50




    }
    static double calcv_0(double vx,double vy, double rot, int acc) {
		/*
			Calcuates the value of v_0 which will esult in the smallest max(new double[]{v_0,v_1,v_2,v_3} );
		*/


        double pos = (Math.sqrt(2)/2);
        double min_range = -Math.sqrt(2);
        double max_range = Math.sqrt(2);
        for (int i = 0; acc > i; i++) {
            //System.out.println("pos:" + pos + " calc:   " + Math.abs(calcSlope_v0(pos,vx,vy,rot ))/calcSlope_v0(pos,vx,vy,rot ) );

            if (calcSlope_v0(pos,vx,vy,rot ) > 0) {
                max_range = pos;
                pos = (max_range + min_range)/2.0;

            }
            else {
                min_range = pos;
                pos = (max_range + min_range)/2.0;
            }
        }
        return pos;

        //calcMotors(vx,vy,rot,v0)



    }
    static double calcSlope_v0(double v0, double vx,double vy, double rot) {
        //System.out.println("Part A" + max(calcMotors(vx,vy,rot,v0+eps)));
        //System.out.println("Part B" + max(calcMotors(vx,vy,rot,v0)));
        return (max(calcMotors(vx,vy,rot,v0+eps)) - max(calcMotors(vx,vy,rot,v0)));
    }
    static double[] calcMotors(double vx,double vy, double rot, double v_0) {
		/*
			Calculate the remaining motors based on v_0
		*/
        double s2 = Math.sqrt(2);
        double v_1 = (vy + (rot/s2) - s2*v_0)/s2;
        double v_2 = -(vx + vy - s2*v_0)/s2;
        double v_3 = (vx + rot/s2 - s2*v_0)/s2;
        return new double[]{v_0,v_1,-v_2,-v_3};
    }
    static double[] calcMotorsFull(double vx,double vy, double rot) {
        double v_0 = calcv_0(vx,vy,rot);
        double[] v_s =  calcMotors(vx,vy,rot,v_0);
        return new double[]{v_s[0],v_s[1],v_s[2],v_s[3]};
    }
    static double[] calcMotorsMax(double vx,double vy, double rot) {
        double v_0 = calcv_0(vx,vy,rot);
        double[] v_s =  calcMotors(vx,vy,rot,v_0);
        double mult = 1.0d/max(v_s);

        return new double[]{v_s[0] * mult,v_s[1] *mult,v_s[2]*mult,v_s[3]*mult};
        //turn motors to max power that they possibly can be at
    }

    /*
    static double test_calcMotors(double vx,double vy, double rot) {
        double[] motors = calcMotors(vx,vy,rot,calcv_0(vx,vy,rot));
        double vx_m = sum_x(motors);
        double vy_m = sum_y(motors);
        double rot_m = sum(motors);
        System.out.println("Diff:" + (vx-vx_m) );
        System.out.println("Diff:" + (vy -vy_m));
        System.out.println("Diff:" + (rot -rot_m));
        return Math.abs((vx-vx_m)) + Math.abs((vy-vy_m)) + Math.abs((rot-rot_m));

    }
    */
    /*
    static void test_calcv_0(double vx,double vy, double rot) {

			//Test to see if calcv_0 is actually running correctly


        System.out.println("vx:" + vx);
        System.out.println("vy:" + vy);
        System.out.println("rot:" + rot);
        double s2 = Math.sqrt(2);
        double[] vals = new double[]{ (vy+ (rot/s2)) /s2, (vx + vy)/s2, (vx + rot/s2)/s2 };
        double v_0 = vals[0] > vals[1] ? ((vals[0] < vals[2]) ? vals[0] : (vals[1] > vals[2]) ? vals[1] : vals[2]) : ((vals[1] < vals[2]) ? vals[1] : ((vals[0] > vals[2]) ? vals[0] : vals[2]));

        for (int i = 0; vals.length > i; i++) {
            System.out.println("v[" + i + "]=" +abs_sum(calcMotors(vx,vy,rot,vals[i])));
        }
        System.out.println("Proposed answer=" +abs_sum(calcMotors(vx,vy,rot,v_0)));


    }
    */

    static double max(double[] vals) {
        double max = Math.abs(vals[0]);
        for (int i = 1; vals.length > i; i++) {
            if (Math.abs(vals[i]) > max) {
                max = Math.abs(vals[i]);
            }
        }
        return max;
    }
    static double sum_x(double[] vals) {
		/*
			Sum absolute values of list
		*/
        double res = 0;
        for (int i = 0; vals.length >i; i++) {
            res += vals[i] * Math.cos( (45 + 90*i) * Math.PI/180.0 );
        }
        return res;
    }
    static double sum_y(double[] vals) {
		/*
			Sum absolute values of list
		*/
        double res = 0;
        for (int i = 0; vals.length >i; i++) {
            res += vals[i] * Math.sin( (45 + 90*i) * Math.PI/180.0 );
        }
        return res;
    }
    static double sum(double[] vals) {
		/*
			Sum absolute values of list
		*/
        double res = 0;
        for (int i = 0; vals.length >i; i++) {
            res += vals[i];
        }
        return res;
    }
    static double abs_sum(double[] vals) {
		/*
			Sum absolute values of list
		*/
        double res = 0;
        for (int i = 0; vals.length >i; i++) {
            res += Math.abs(vals[i]);
        }
        return res;
    }
}