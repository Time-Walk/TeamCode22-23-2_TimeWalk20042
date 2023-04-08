package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="СН-СП-0: Терминал > Парковка по цвету", group="blue")
@Disabled
public class FixedBlueTerminalParking extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.KL.setPosition(R.OPENPOS);

        while ( !isStarted() ) {
            R.MgI = 0;
            R.GrI = 0;
            R.CnI = 0;

            telemetry.addData("State", "Detecking");
            telemetry.update();

            for (int i = 250; i < 390; i = i + 10) {
                Bitmap img_ = R.getImage();
                img_ = R.setPixels_(img_, 250, 140, 150, 3, 220, 70, 120);
                img_ = R.setPixels_(img_, 250, 140, 210, 3, 50, 120, 50);
                img_ = R.setPixels_(img_, 250, 3, 150, 60, 50, 120, 50);
                img_ = R.setPixels_(img_, 390, 3, 150, 60, 123, 123, 123);
                FtcDashboard.getInstance().sendImage(img_);
                for (int l = 150; l < 210; l = l + 10) {
                    int Red = Color.red(R.getImage().getPixel(i, l));
                    int Green = Color.green(R.getImage().getPixel(i, l));
                    int Blue = Color.blue(R.getImage().getPixel(i, l));
                    R.analyze(Red, Green, Blue);
                    telemetry.addData("i", i);
                    telemetry.addData("l", l);
                    telemetry.update();
                }
            }
            telemetry.addData("Exit from for.", R.MgI);
        }


        waitForStart();
        R.KL.setPosition(R.CLOSEPOS);
        R.stabilizeKL();

        //okay, let's go!

        //R.setMtPower(0.25, 0.25, -0.22, 0.22);

        String s = "Ns";
        if (R.MgI > R.GrI && R.MgI > R.CnI) {
            s = "Mg";
            telemetry.addData("Color", "Mg");
            telemetry.addData("mg", "███╗░░░███╗░██████╗░");
            telemetry.addData("mg", "████╗░████║██╔════╝░");
            telemetry.addData("mg", "██╔████╔██║██║░░██╗░");
            telemetry.addData("mg", "██║╚██╔╝██║██║░░╚██╗");
            telemetry.addData("mg", "██║░╚═╝░██║╚██████╔╝");
            telemetry.addData("mg", "╚═╝░░░░░╚═╝░╚═════╝░");
        }
        if (R.GrI > R.MgI && R.GrI > R.CnI) {
            s = "Gr";
            telemetry.addData("Color", "Gr");
            telemetry.addData("gr", "░██████╗░██████╗░");
            telemetry.addData("gr", "██╔════╝░██╔══██╗");
            telemetry.addData("gr", "██║░░██╗░██████╔╝");
            telemetry.addData("gr", "██║░░╚██╗██╔══██╗");
            telemetry.addData("gr", "╚██████╔╝██║░░██║");
            telemetry.addData("gr", "░╚═════╝░╚═╝░░╚═╝");
        }
        if (R.CnI > R.GrI && R.CnI > R.MgI) {
            s = "Cn";
            telemetry.addData("Color", "Cn");
            telemetry.addData("cn", "░█████╗░███╗░░██╗");
            telemetry.addData("cn", "██╔══██╗████╗░██║");
            telemetry.addData("cn", "██║░░╚═╝██╔██╗██║");
            telemetry.addData("cn", "██║░░██╗██║╚████║");
            telemetry.addData("cn", "╚█████╔╝██║░╚███║");
            telemetry.addData("cn", "░╚════╝░╚═╝░░╚══╝");
        }
        telemetry.addData("Mg count", R.MgI);
        telemetry.addData("Gr count", R.GrI);
        telemetry.addData("Cn count", R.CnI);
        telemetry.addData("SMACHNAYA", "PAPERDELINA");
        telemetry.update();
        R.delay(100);

        if (s != "Cn" ) {
            R.setMtPower(0.3, 0.3, -0.3, -0.3);
            R.delay(270);
            R.setMtZero();
            R.delay(220);
            R.rotate(85);
            R.delay(200);
            R.Katet(60, 1);
            R.stabilizeKL();
            R.KL.setPosition(R.OPENPOS);
            R.delay(1000);
            R.Katet(42, 3);
            R.KL.setPosition(R.CLOSEPOS);
            R.rotate(-90);
            R.setMtPower(-0.3, -0.3, 0.3, 0.3);
            R.delay(1000);
            R.setMtZero();
            R.delay(500);
            R.setMtPower(0.3, 0.3, -0.3, -0.3);
            R.delay(150);
            R.setMtZero();
            R.delay(200);
        }



        if (s == "Ns") {
            R.Katet(60, 4);
        }
        if (s == "Cn") {
            R.setMtPower(0.3, 0.3, -0.3, -0.3);
            R.delay(75);
            R.setMtZero();
            R.delay(300);
            R.katetPlus(220, 2, 0.4, 0.135, true);
            R.delay(200);
            R.setMtPower(-0.3, 0, 0, 0.3);
            R.delay(1000);
            R.setMtZero();
            R.delay(200);
            R.katetPlus(140, 1, 0.55, 0.2, true);
            R.delay(200);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(1000);
            R.setMtZero();
            R.delay(200);
            R.setMtPower(-0.3, 0.3, -0.3, 0.3);
            R.delay(500);
            R.setMtZero();
            R.delay(300);
            R.rotate(-175);
            // R.delay(400);
            // R.rotateS(85);
            R.delay(300);
            R.setMtPower(-0.3, 0.3, -0.3, 0.3);
            R.delay(1100);
            R.setMtZero();
            R.delay(200);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(450);
            R.setMtZero();
            R.delay(200);
            R.katetPlus(163, 1, 0.7, 0.2, false);
            R.delay(200);
            R.stabilizeKL();
            R.KL.setPosition(R.OPENPOS);
            R.delay(500);
            R.katetPlus(120, 3, 0.7, 0.175, false);
            R.delay(400);
            R.KL.setPosition(R.CLOSEPOS);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(400);
            R.setMtZero();
            R.delay(500);
        }
        if (s == "Gr") {
            R.Katet(80, 1);
        }
        if (s == "Mg") {
            R.stabilizeKL();
            R.delay(500);
            R.Katet(90, 4);
            R.delay(200);
            R.setMtPower(-0.3, -0.3, 0.3, 0.3);
            R.delay(400);
            R.setMtZero();
            R.delay(200);
            R.Katet(90, 1);
            R.delay(200);
            R.setMtPower(-0.3, 0.3, -0.3, 0.3);
            R.delay(450);
            R.setMtZero();
            R.delay(500);
        }

    }

}
