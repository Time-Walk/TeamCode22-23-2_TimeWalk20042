package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="КР-СЛ-0: Терминал > Парковка по цвету", group="red")
@Disabled
public class FixedRedTerminalParking extends LinearOpMode {

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

        if (s != "Mg" ) {
            R.setMtPower(0.3, 0.3, -0.3, -0.3);
            R.delay(200);
            R.setMtZero();
            R.delay(200);
            R.rotate(-90);
            R.delay(200);
            R.Katet(70, 1);
            R.stabilizeKL();
            R.KL.setPosition(R.OPENPOS);
            R.delay(1000);
            R.Katet(52, 3);
            R.KL.setPosition(R.CLOSEPOS);
            R.rotate(90);
            R.setMtPower(-0.3, -0.3, 0.3, 0.3);
            R.delay(800);
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
            R.delay(500);
            R.Katet(100, 2);
            R.delay(200);
            R.setMtPower(-0.3, -0.3, 0.3, 0.3);
            R.delay(400);
            R.setMtZero();
            R.delay(200);
            R.Katet(100, 1);
        }
        if (s == "Gr") {
            R.Katet(90, 1);
        }
        if (s == "Mg") {
            R.setMtPower(0.3, 0.3, -0.3, -0.3);
            R.delay(125);
            R.setMtZero();
            R.delay(300);
            R.katetPlus(145, 4, 0.4, 0.135, true);
            R.delay(200);
            R.setMtPower(-0.3, -0.3, 0.3, 0.3);
            R.delay(400);
            R.setMtZero();
            R.delay(200);
            R.katetPlus(130, 1, 0.55, 0.2, true);
            R.delay(200);
            R.setMtPower(-0.3, 0.3, -0.3, 0.3);
            R.delay(1000);
            R.setMtZero();
            R.delay(200);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(500);
            R.setMtZero();
            R.delay(300);
            R.rotate(185);
            // R.delay(400);
            // R.rotateS(85);
            R.delay(200);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(1300);
            R.setMtZero();
            R.delay(200);
            R.setMtPower(-0.3, 0.3, -0.3, 0.3);
            R.delay(350);
            R.setMtZero();
            R.delay(200);
            R.katetPlus(82, 1, 0.8, 0.2, false);
            R.delay(200);
            R.stabilizeKL();
            R.KL.setPosition(R.OPENPOS);
            R.delay(500);
            R.katetPlus(110, 3, 0.7, 0.175, false);
            R.delay(400);
            R.KL.setPosition(R.CLOSEPOS);
            R.setMtPower(0.3, -0.3, 0.3, -0.3);
            R.delay(400);
            R.setMtZero();
            R.delay(500);
        }

    }

}
