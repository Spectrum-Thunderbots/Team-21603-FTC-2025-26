/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// autonomus expecting a disitance sensor on the left right and back with the camera on the launcher, robot starting facing back towards the wall

package org.firstinspires.ftc.teamcode;




import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
@Autonomous(name="Start far Blue", group="Linear OpMode")

public class farParkBlue extends LinearOpMode {

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor flyWheelLeft = null;
    private DcMotor flyWheelRight = null;


    private Servo aimServoleft = null;

    private Servo pusherServo1 = null;
    private Servo pusherServo2 = null;



    double encoderClicksPerWheelRev = 415.2;
    double wheelDiam = 104;
    double wheelCircumference = wheelDiam * Math.PI;
    double clicksPerMM = encoderClicksPerWheelRev/wheelCircumference;


    float tolerance = 1;


    String currentStepDescription = "ERROR";
    double targetYaw = 0;
    double robotYaw = 0;
    @Override
    public void runOpMode() {
        hardwareInit();

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetryUpdate();

        waitForStart();
        runtime.reset();

        step1();

        step2();

        step3();

        step4();

        step5();

        step6();

    }

    public void step1(){

        targetYaw = 0;

        currentStepDescription = "move away from wall";

        telemetryUpdate();

        encoderDrive(.2, 200, 200, 2.5, false);

    }
    public void step2(){

        currentStepDescription = "turn towards target";

        double diff;

        targetYaw = -21.28;


        while (targetYaw >= robotYaw + tolerance || targetYaw + tolerance <= robotYaw) {

            robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            diff = robotYaw - targetYaw;

            diff /= 300;

            diff = Math.max(-0.5, Math.min(diff, 0.5));




            frontLeftDrive.setPower(-diff);
            frontRightDrive.setPower(diff);
            backLeftDrive.setPower(-diff);
            backRightDrive.setPower(diff);

            telemetryUpdate();

        }

    }
    public void step3(){

        flyWheelLeft.setPower(.5);
        flyWheelRight.setPower(.5);

        aimServoleft.setPosition(.4);

        sleep(1000);

        for(int i = 1; i == 3; i++) {

            currentStepDescription = "launching ball" + i;

            telemetryUpdate();


            for(double c = 0; c == .5; c += .1) {
                pusherServo2.setPosition(c);
                pusherServo1.setPosition(1 - c);

                telemetryUpdate();

                sleep(100);
            }

            sleep(500);

            for(double c = .5;  c == 0; c -= .1){
                pusherServo2.setPosition(c);
                pusherServo1.setPosition(1-c);

                telemetryUpdate();

                sleep(100);
            }

            sleep(500);

        }

        flyWheelLeft.setPower(0);
        flyWheelRight.setPower(0);


    }
    public void step4(){

        currentStepDescription = "drive towards parking zone";

        telemetryUpdate();

        encoderDrive(.2, 581, 581, 5, false);

    }
    public void step5(){

        currentStepDescription = "drive towards parking zone (cont)";

        telemetryUpdate();

        encoderDrive(.2, -365, -365, 5, true);

    }
    public void step6(){

        double diff;

        targetYaw = 0;

        currentStepDescription = "turn into parking zone";


        while (opModeIsActive()) {

            robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            diff = robotYaw - targetYaw;

            diff /= 300;

            diff = Math.max(-0.5, Math.min(diff, 0.5));


            frontLeftDrive.setPower(-diff);
            frontRightDrive.setPower(diff);
            backLeftDrive.setPower(-diff);
            backRightDrive.setPower(diff);

            telemetryUpdate();
        }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void encoderDrive(double speed, double leftMM, double rightMM, double timeoutS,boolean strafe) {
        int newLeftTarget =67;
        int newRightTarget =67;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            newLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (leftMM * clicksPerMM);
            newRightTarget = frontRightDrive.getCurrentPosition() + (int) (rightMM * clicksPerMM);


            if(strafe) {
                // if strafe is true set strafing  values to the positions

                newLeftTarget *= 2;
                newRightTarget *= 2;

                speed *= 2;

                frontLeftDrive.setTargetPosition(-newLeftTarget);
                backLeftDrive.setTargetPosition(newLeftTarget);

                frontRightDrive.setTargetPosition(newRightTarget);
                backRightDrive.setTargetPosition(-newRightTarget);



            }
            else{
                // Determine new target position, and pass to motor controller

                frontLeftDrive.setTargetPosition(newLeftTarget);
                backLeftDrive.setTargetPosition(newLeftTarget);

                frontRightDrive.setTargetPosition(newRightTarget);
                backRightDrive.setTargetPosition(newRightTarget);

            }

            speed = Math.min(1, Math.max(speed, -1));

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (frontRightDrive.isBusy() || backRightDrive.isBusy())) {

                telemetryUpdate();

            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void hardwareInit(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));



        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        flyWheelLeft = hardwareMap.get(DcMotor.class, "Motor0");
        flyWheelRight = hardwareMap.get(DcMotor.class, "Motor1");

        flyWheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flyWheelRight.setDirection(DcMotor.Direction.FORWARD);

        flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        aimServoleft = hardwareMap.get(Servo.class, "Servo0");

        pusherServo1 = hardwareMap.get(Servo.class, "CRServo0");
        pusherServo2 = hardwareMap.get(Servo.class, "CRServo1");

        imu.resetYaw();
    }
    public void telemetryUpdate(){

        telemetry.addData("Run Time", runtime.seconds());

        telemetry.addLine();
        telemetry.addData("oreintation", robotYaw);
        telemetry.addData("target oreintation", targetYaw);

        telemetry.addLine();
        telemetry.addData("liniar actuator position", aimServoleft.getPosition());
        telemetry.addData("left pusher servo position", pusherServo1.getPosition());
        telemetry.addData("right pusher servo position", pusherServo2.getPosition());
        telemetry.addLine();
        telemetry.addLine(currentStepDescription);
        telemetry.update();

    }
}