package us.ftcteam11574.teamcode2020;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpTest", group="Iterative Opmode")
public class ServoClass extends OpMode {
    private Servo Gate = null;

    @Override
    public void init() {
        Gate = hardwareMap.get(Servo.class, "Gate");
        telemetry.addData("Servo",Gate.getPortNumber());
        telemetry.addData("Servo 2",Gate.getPosition());
        telemetry.addData("Servo 3",Gate.getConnectionInfo());
        Gate.setPosition(0);


    }

    @Override
    public void loop() {

    }
}
