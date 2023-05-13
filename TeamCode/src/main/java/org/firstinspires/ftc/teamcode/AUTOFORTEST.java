package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Автоном для теста", group="")
//@Disable
public class AUTOFORTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();

        //telemetry = R.ftcDash.getTelemetry();

        /*telemetry.addData("LT", 0);
        telemetry.addData("Err", 0);
        telemetry.addData("P", 0);
        telemetry.addData("rI", 0);
        telemetry.addData("I", 0);
        telemetry.addData("ErD", 0);
        telemetry.addData("D", 0);
        telemetry.addData("pwf", 0);
        telemetry.update();*/

        waitForStart();

        //okay, let's go!

        /*boolean dLeft = true, dRight = true, dDown = true, dUp = true;
        while (opModeIsActive()) {
            if ( gamepad1.x ) {
                R.g(500);
            }
            if ( gamepad1.y ) {
                R.g(-500);
            }
            if ( gamepad1.dpad_left ) { if (dLeft ) { R.kp -= 0.05; dLeft=false; }} else { dLeft=true; }
            if ( gamepad1.dpad_right) { if ( dRight ) { R.kp += 0.05; dRight=false; }} else { dRight=true; }
            if ( gamepad1.dpad_down ) { if ( dDown ) { R.kd -= 0.05; dDown=false; }} else { dDown=true; }
            if ( gamepad1.dpad_up ) { if ( dUp ) { R.kd += 0.05; dUp=false; }} else { dUp=true; }
            telemetry.addData("kp", R.kp);
            telemetry.addData("kd", R.kd);
            telemetry.update();
        }*/

        /*R.g(400);
        R.delay(1000);
        R.g(-400);
        R.delay(1000);
        R.g(150);
        R.delay(1000);
        R.g(200);
        R.delay(1000);
        R.g(-350);
        R.delay(1000);*/

        //Low - 240
        //Medium - 390
        //High - 545

        /*R.setLift(545);
        R.LT.setPower(0.0008);
        R.delay(1000);
        telemetry.addData("LT", R.LT.getCurrentPosition());
        telemetry.update();
        R.delay(10000);*/

        R.g(1000);
        R.delay(2000);
        R.g(-1000);

    }

}
