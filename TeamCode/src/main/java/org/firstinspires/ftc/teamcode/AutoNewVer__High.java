package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="0-0480-Высокий__КР-СП__СН-СЛ", group="New")
public class AutoNewVer__High extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.initNeonController.start();

        R.KL.setPosition(R.OPENPOS);
        R.stabilizeKL();

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
        R.stabilizeKL();

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

        R.setMtPower(0.3, 0.3, -0.3, -0.3);
        R.delay(700);
        R.setMtZero();
        R.rotateS(45);
        R.g(-1050);
        R.rotate(-90);
        R.g(2050);
        R.liftUp();
        R.LT.setPower(R.HOLDPOWER);
        R.rotate(90);
        R.g(-700);
        R.rotateS(-45);
        R.setMtPower(-0.35, -0.35, 0.35, 0.35);
        R.delay(200);
        R.setMtZero();
        R.KL1.setPosition(R.ROTPOS);
        R.delay(1000);
        R.KL1.setPosition(R.DEFPOS);
        R.setMtPower(0.35, 0.35, -0.35, -0.35);
        R.delay(200);
        R.setMtZero();
        R.rotateS(45);
        R.g(700);
        R.rotate(-90);
        R.LT.setPower(-0.01);
        R.delay(1500);
        R.LT.setPower(0);
        R.g(-1000);
        R.rotate(90);

    }

}
