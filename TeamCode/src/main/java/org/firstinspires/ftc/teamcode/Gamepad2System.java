

package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;



//@Disabled
@TeleOp(name="Gamepad2System", group="Linear OpMode")

public class Gamepad2System extends LinearOpMode {

    private DcMotor intake = null;


    private CRServo pusherServo = null;

    @Override
    public void runOpMode() {


        DcMotor Shoot2 = hardwareMap.dcMotor.get("Shoot2");
        DcMotor Shoot1 = hardwareMap.dcMotor.get("Shoot1");

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        CRServo pusherServo = hardwareMap.crservo.get("CRServo0");


        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        // Wait for the game to start (driver presses START)

        waitForStart();


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {


            if (gamepad2.a) {
                intake.setPower(1);
            } else if (gamepad2.b) {
                intake.setPower(-1);
            } else intake.setPower(0);


            if (gamepad2.dpad_up) {
                pusherServo.setPower(1);
            } else if (gamepad2.dpad_down) {
                pusherServo.setPower(-1);
            } else pusherServo.setPower(0);


            if (gamepad2.leftBumperWasPressed()) {
                Shoot1.setPower(1);
                Shoot2.setPower(1);
            } else if (gamepad2.rightBumperWasPressed()) {
                Shoot1.setPower(0);
                Shoot2.setPower(0);
            }


            telemetry.update();

        }
    }
}