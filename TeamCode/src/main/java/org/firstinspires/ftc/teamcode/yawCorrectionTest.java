

package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
@TeleOp(name="yawCorrectionTest", group="Linear OpMode")

public class yawCorrectionTest extends LinearOpMode {

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




    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;




    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward







        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);







        float driveX = 0;
        float driveY = 0;
        float driveTurn = 0;


        double targetYaw = 0;

        float tolerance = 10;

        double robotYaw = 0;

        double diff = 0;


        // Wait for the game to start (driver presses START)

        waitForStart();
        runtime.reset();

        imu.resetYaw();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {

            robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);








            if(gamepad1.dpadUpWasPressed()) tolerance += 1;
            else if (gamepad1.dpadDownWasPressed()) tolerance -= 1;

            if (gamepad1.aWasPressed()){ targetYaw += 45;}
            else if (gamepad1.bWasPressed()) targetYaw -= 45;

            if (gamepad1.b) targetYaw = 0;

            if (gamepad1.right_stick_x != 0){

                targetYaw += gamepad1.right_stick_x/2;

                if (targetYaw != (Math.max(-180, Math.min(targetYaw, 180)))){

                    if (targetYaw > 180){

                        targetYaw -=360;

                    }
                    else{

                        targetYaw += 360;

                    }

                }




            }




            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //sets gyro correction and any actuator power

            telemetry.update();
            //gyro correction
            if(targetYaw >= robotYaw+tolerance || targetYaw+tolerance <= robotYaw){
                diff = robotYaw -targetYaw;

                diff /= 500;

                diff = Math.max(-0.5, Math.min(diff, 0.5));



                driveTurn += (float) diff;


            }
            else{driveTurn = 0;}









            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -driveY;  // Note: pushing stick forward gives negative value
            double lateral =  driveX;
            double yaw     =  driveTurn;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Show the elapsed game time
            telemetry.addData("Run Time", runtime.seconds());
            telemetry.addData("oreintation", robotYaw);
            telemetry.addData("difference", diff);
            telemetry.addData("target oreintation", targetYaw);
            telemetry.addData("added movement", driveTurn);
            telemetry.addData("Tolerance", tolerance);

            telemetry.update();
        }

    }

}




