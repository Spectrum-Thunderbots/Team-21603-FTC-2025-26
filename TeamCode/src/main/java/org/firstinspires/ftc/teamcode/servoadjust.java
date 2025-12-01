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

    double varileft = 0;
    double variright = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        linearActuatorServo1 = hardwareMap.get(Servo.class, "CRServo0");
        linearActuatorServo2 = hardwareMap.get(Servo.class, "CRServo1");




        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                varileft += .1;
                variright -= .1;
            } else if (gamepad1.dpadDownWasPressed()) {

                varileft -= .1;
                variright += .1;
            }

            varileft = Math.max(0, Math.min(varileft, .6));
            variright = Math.max(.4, Math.min(variright, 1));

            linearActuatorServo1.setPosition(variright);
            linearActuatorServo2.setPosition(varileft);


            telemetry.addData("DAtaIn", varileft);
            telemetry.addData("datain", variright);
            telemetry.update();

        }
    }

}