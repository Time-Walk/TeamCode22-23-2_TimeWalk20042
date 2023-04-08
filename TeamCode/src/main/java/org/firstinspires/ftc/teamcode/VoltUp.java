package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Тест подъема лифта с вольтметром", group="")
public class VoltUp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        double V = R.vs.getVoltage();

        double k = 0.02 * V + 9.07;
        int ik = (int) k;
        R.LB.setPower(0.3);
        R.delay(500*ik);
        R.LB.setPower(0);
        R.telemetry.addData("Voltage", R.vs.getVoltage());
        R.telemetry.addData("k", k);
        R.telemetry.update();
        R.delay(10000);


    }

}
