package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

/**
 * Control scheme
 *     gamepad1:
 *     -left stick: moving forwards backwards left right
 *     -right stick: turning
 *
 *     gamepad2:
 *     -right bumper: override turning to lock onto an april tag
 *     -dpad left/right: activate ball pushers
 *     -b: turns on/off the flywheels
 *     -dpad up/down: changes the flywheel power
 *     -a/y: changes launcher angle
 */

//@Disabled
@TeleOp

public class mainRobotCode extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


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




    // Declare OpMode members for each of the 4 driving motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor flyWheelLeft = null;
    private DcMotor flyWheelRight = null;


    private Servo aimServoleft = null;
    private Servo aimServoRight = null;

    private Servo pusherServo1 = null;
    private Servo pusherServo2 = null;

    private DistanceSensor leftSensorDistance;
    private DistanceSensor rightSensorDistance;
    private DistanceSensor backSensorDistance;



    @Override
    public void runOpMode() {


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));




        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");//port 0
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");//port 1
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");//port 2
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");//port 3

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);



        flyWheelLeft = hardwareMap.get(DcMotor.class, "Motor0");//expansion port 0
        flyWheelRight = hardwareMap.get(DcMotor.class, "Motor1");//expansion port 1

        flyWheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flyWheelRight.setDirection(DcMotor.Direction.FORWARD);





        flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        aimServoleft = hardwareMap.get(Servo.class, "Servo0");//port 0
        aimServoRight = hardwareMap.get(Servo.class, "Servo1");//port 1

        pusherServo1 = hardwareMap.get(Servo.class, "CRServo0");//port 2
        pusherServo2 = hardwareMap.get(Servo.class, "CRServo1");//port 3

        backSensorDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor0");
        rightSensorDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        leftSensorDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor2");



        initAprilTag();


        float driveX;
        float driveY;
        float driveTurn = 0;


        double targetYaw = 0;

        float tolerance = 1;

        double robotYaw = 0;

        double diff;




        boolean flywheelState = false;

        double flyWheelPow = .5;

        double servoAngle = .2;


        pusherServo2.setPosition(0.4);
        pusherServo1.setPosition(.6);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        imu.resetYaw();
        /* run until the end of the match (driver presses STOP) ********************************************************************************************/
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





















            if (gamepad2.dpad_left) {
                pusherServo1.setPosition(1);
                pusherServo2.setPosition(0);
            }
            else if (gamepad2.dpad_right){
                pusherServo1.setPosition(.6);
                pusherServo2.setPosition(.4);
            }
            if(gamepad2.dpadUpWasPressed()){
                flyWheelPow += .05;
            } else if (gamepad2.dpadDownWasPressed()) {
                flyWheelPow-= .05;
            }





















            /* ***********************************************************************************************************************************************/

            driveX = gamepad1.left_stick_x;
            driveY = gamepad1.left_stick_y;


            driveTurn = gamepad1.right_stick_x;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -driveY;  // Note: pushing stick forward gives negative value
            double lateral = driveX;
            double yaw = driveTurn;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > .25) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;


                targetYaw = robotYaw;
            }
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            /* driving code above *********************************************************************************/


































            if (gamepad2.yWasPressed()) {
                servoAngle += .05;

            } else if (gamepad2.aWasPressed()) {

                servoAngle -= .05;
            }


            if(gamepad2.bWasPressed()){
                flywheelState = !flywheelState;
            }


            if(flywheelState) {
                flyWheelLeft.setPower(flyWheelPow);
                flyWheelRight.setPower(flyWheelPow);
            }
            else {
                flyWheelLeft.setPower(0);
                flyWheelRight.setPower(0);
            }




            /* sets power to actuators **********************************/

            servoAngle = Math.max(0.3, Math.min(.6, servoAngle));

            aimServoleft.setPosition(servoAngle);
            aimServoRight.setPosition(servoAngle);





            /* telemetry updates ***************************************/
            //telemetryAprilTag();
            telemetry.addData("fly Wheel Power", flyWheelPow);
            telemetry.addData("launcher angle", servoAngle);
            telemetry.addData("pusher servo", pusherServo1.getPosition());
            telemetry.addData("target yaw", targetYaw);
            telemetry.addData("robot yaw", robotYaw);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));



        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");


    }

    public Double aprilTagBearing() {

        double bearingIn =0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if(detection.id == 20 || detection.id == 24) {
                    bearingIn = detection.ftcPose.bearing;
                }
            }
        }


        return bearingIn;
    }
}

