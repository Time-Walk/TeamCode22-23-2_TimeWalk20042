package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="0-0480-Низкий(слева)--", group="New")
public class AutoNewVer__Low extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.initNeonController.start();

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
        //R.AutoNeonController.start();

        R.KL.setPosition(R.CLOSEPOS);

        //okay, let's go!

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
            R.NeState = "Plink";
            R.plinkCount = 1;
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
            R.NeState = "Plink";
            R.plinkCount = 2;
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
            R.NeState = "Plink";
            R.plinkCount = 3;
        }
        telemetry.addData("Mg count", R.MgI);
        telemetry.addData("Gr count", R.GrI);
        telemetry.addData("Cn count", R.CnI);
        telemetry.update();
        R.delay(100);

        R.stabilizeKL();
        R.setLift(300);
        R.LT.setPower(0.0008);
        R.setMtPower(0.4, 0.4, -0.4, -0.4);
        R.delay(500);
        R.setMtZero();
        R.rotateS(-55);
        R.setMtPower(-0.35, -0.35, 0.35, 0.35);
        R.delay(400);
        R.setMtZero();
        R.KL.setPosition(R.OPENPOS);
        R.delay(2000);
        R.KL.setPosition(R.CLOSEPOS);

        if ( s == "Mg" ) {
            R.rotate(90);
            R.g(-1200);
            R.rotate(-90);
            R.g(1700);
            R.setLift(-190);
            //R.LT.setPower(-0.1);
            //R.delay(600);
            //R.LT.setPower(0);
        }

        if ( s == "Gr" ) {
            R.setLift(-190);
            R.KL1.setPosition(R.DEFPOS-0.1);
            R.g(1700);
            R.setMtPower(0, -0.35, 0.35, 0);
            R.delay(400);
            R.setMtZero();
            R.rotateS(45);
            //R.LT.setPower(-0.1);
            //R.delay(600);
            //R.LT.setPower(0);
        }

        if ( s == "Cn" || s == "Ns") {
            R.rotate(90);
            R.g(1300);
            R.rotate(-90);
            R.g(1700);
            R.setLift(-190);
            //R.LT.setPower(-0.1);
            //R.delay(600);
            //R.LT.setPower(0);
        }

    }

}