package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Проверка энкодеров", group="")
public class CheckEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        while ( !isStopRequested() ) {
            telemetry.addData("LF", R.LF.getCurrentPosition());
            telemetry.addData("LT", R.LT.getCurrentPosition());
            telemetry.update();
        }

    }

}
