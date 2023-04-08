package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Гипотеза", group="")
public class peepoGipoteza extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.gamepad_init(gamepad1,gamepad2);
        waitForStart();

        //okay, let's go!
        double p0 = 0, V0 = 0, v0 = 0, p = 0, V = 0;
        R.liftControllerT.start();  //Запуск работы лифта
        while (!isStopRequested()){
            R.wheelbase();
            R.servoControllerPro();
            R.servoController();
            if ( gamepad1.y ) {
                p0 = 0.7;
                V0 = R.vs.getVoltage();
                R.RB.setPower(p0);
                R.delay(500);
                R.RB.setPower(0);
                v0 = p0*V0;
                telemetry.addData("p0", p0);
                telemetry.addData("V0", V0);
                telemetry.addData("v0", v0);
                telemetry.update();
            }
            if ( gamepad1.x ) {
                V = R.vs.getVoltage();
                p = v0 / V;
                R.RB.setPower(p);
                R.delay(500);
                R.RB.setPower(0);
                telemetry.addData("p0", p0);
                telemetry.addData("V0", V0);
                telemetry.addData("v0", v0);
                telemetry.addData("V", V);
                telemetry.addData("p", p);
                telemetry.update();
            }
        }

    }

}
