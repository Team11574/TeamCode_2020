package us.ftcteam11574.teamcode2020.Examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*

    This could help

 */

@TeleOp(name="IMU test", group="Linear Opmode")
public class IMUTEST extends OpMode {
    public static BNO055IMU imu;
    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        telemetry.addData("Angle",imu.getAngularOrientation());
        telemetry.addData("Position",imu.getPosition());
        telemetry.addData("Velocity", imu.getVelocity());
        telemetry.update();
    }
}
