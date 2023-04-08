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

        R.g(400);
        R.delay(1000);
        R.g(-400);
        R.delay(1000);
        R.g(150);
        R.delay(1000);
        R.g(200);
        R.delay(1000);
        R.g(-350);
        R.delay(1000);

    }

}
