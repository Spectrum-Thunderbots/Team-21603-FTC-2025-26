package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class servoadjust extends  LinearOpMode {
    private Servo linearActuatorServo1 = null;
    private Servo linearActuatorServo2 = null;
    double rectracted = .3;
    double extended = .7;
    double mid = 0.1;

    double vari = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        linearActuatorServo1 = hardwareMap.get(Servo.class, "Servo0");
        linearActuatorServo2 = hardwareMap.get(Servo.class, "Servo1");




        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                vari += .1;

            } else if (gamepad1.dpadDownWasPressed()) {

                vari -= .1;
            }

            vari = Math.max(0.4, Math.min(vari, 0.7));
            linearActuatorServo1.setPosition(vari);
            linearActuatorServo2.setPosition(vari);


            telemetry.addData("DAtaIn", vari);
            telemetry.update();

        }
    }

}