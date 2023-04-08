package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TeleOp")
public class teleopo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot2021 R = new Robot2021();
        R.initFields(dashboardTelemetry, this, hardwareMap);
        R.init();
        R.gamepad_init(gamepad1,gamepad2);
        waitForStart();
        R.liftControllerT.start();
        while (!isStopRequested()){
            R.UP.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            R.wheelbase();
            R.servoController();
            R.smartRotate();
            R.valController();
            //R.DEBUG();
            //telemetry.addData("Test", R.testVuforia());
            /*dashboardTelemetry.addData("Width", R.getImage().getWidth());
            dashboardTelemetry.addData("Height", R.getImage().getHeight());
            dashboardTelemetry.addData("Red", Color.red(R.getImage().getPixel(500, 250)));
            dashboardTelemetry.addData("Green", Color.green(R.getImage().getPixel(500, 250)));
            dashboardTelemetry.addData("Blue", Color.blue(R.getImage().getPixel(500, 250)));
            dashboardTelemetry.addData("CONUS", R.CoNuS(200, 125, 100, 0.05, Color.red(R.getImage().getPixel(500, 250)), Color.green(R.getImage().getPixel(500, 250)), Color.blue(R.getImage().getPixel(500, 250))));
            dashboardTelemetry.update();
            R.stream();*/
        }

    }
}
