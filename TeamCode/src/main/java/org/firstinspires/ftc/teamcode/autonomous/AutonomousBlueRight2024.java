package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.JCTree;

//import org.firstinspires.ftc.robotcontroller.internal.PHI;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name = "AutoBlueRight2024", group = "Linear Opmode")
public class AutonomousBlueRight2024 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Timer timer = new Timer();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor launcher;
    private boolean targetFound = false;
    private AprilTagDetection desiredTag = null;
    private int DESIRED_TAG_ID = 0; //change for which tag depending on position

    private Servo clawL;
    private Servo clawR;

    private double clawLClose = 0.85;
    private double clawRClose = 0.000001;

    private double clawLOpen = 0.375;
    private double clawROpen = 0.625;

    private Servo armServo;
    private Servo clawRotation;
    private DcMotorEx lift;
    private DcMotorEx arm;
    private DigitalChannel pSensor;
    private DistanceSensor sensor;
    private DistanceSensor distanceSensor;
    private Servo servo;

    private boolean pos1;
    private boolean pos2;
    private boolean pos3;

    private double motorPower = 0.75;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };



    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP... maybe 1??
    static final double WHEEL_DIAMETER_INCHES = 10 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (12 / 9.6)/* (12/10.25) */ * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //did I insert that in there just for adjustment?

    private static final String TFOD_MODEL_ASSET = "model_20231220_191043.tflite";
    private String label;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        while (opModeInInit()) { // Tell the driver that initialization is complete.
            //init everything

            //initTfod();

            telemetry.addData("Status", "Initializing ...");
            telemetry.update();

            clawL = hardwareMap.get(Servo.class, "clawL");
            clawR = hardwareMap.get(Servo.class, "clawR");
            armServo = hardwareMap.get(Servo.class, "armServo");

            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");

            lift = hardwareMap.get(DcMotorEx.class, "lift");
            arm = hardwareMap.get(DcMotorEx.class, "arm");

            servo = hardwareMap.get(Servo.class, "servo");

            sensor = hardwareMap.get(DistanceSensor.class, "sensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

            telemetry.addData("Lift current position: ", lift.getCurrentPosition());
            telemetry.update();

            //imu
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

            imuParameters.mode = BNO055IMU.SensorMode.IMU;
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(imuParameters);
            initAprilTag();

            //label = telemetryTfod();

            //apriltag
            //telemetryAprilTag();

            // Share the CPU.
            clawL.setPosition(clawLClose);
            clawR.setPosition(clawRClose);
//            encoderDrive(-1000, -0.5);
        }

        waitForStart();

        //after start
        while(opModeIsActive()) {
            //detect spike mark
//            sleep(100);
//            armEncoderDrive(600, 0.42);
//            sleep(500);
//            goForwardEncoder(16, .2);
//            distanceSensorOption();
//
//            //place yellow pixel down
//            if (pos2) {
//                goForwardEncoder(6.25, 0.2);
//                liftEncoderDrive(1750, 1);
//                armEncoderDrive(265, .42);
//                sleep(500);
//                clawL.setPosition(clawLOpen);
//                sleep(500);
//                goBackwardEncoder(4, 0.2);
//                DESIRED_TAG_ID = 2;
//            } else if (pos3) {
//                goForwardEncoder(4, 0.2);
//                rotate(-33.3, 0.2);
//                liftEncoderDrive(2000, 1);
//                armEncoderDrive(265, 0.42);
//                sleep(500);
//                clawL.setPosition(clawLOpen);
//                sleep(500);
//                rotate(2.3, 0.2);
//                DESIRED_TAG_ID = 3;
//            } else if (pos1) {
//                goForwardEncoder(4, 0.2);
//                rotate(33.3, 0.75);
//                liftEncoderDrive(1750, 1);
//                armEncoderDrive(265, 0.42);
//                sleep(500);
//                clawL.setPosition(clawLOpen);
//                sleep(500);
//                rotate(-2.3, 0.2);
//                DESIRED_TAG_ID = 1;
//            }
//
//
//            //go to the board
//            liftEncoderDrive(1000, 1);
//            goBackwardEncoder(12, 0.2);
//            rotate(-81.25, 0.4);
            goBackwardEncoder(86, 0.4);

            //detect the board
            while ((distanceSensor.getDistance(DistanceUnit.INCH) >= 23.75 && opModeIsActive())) {
                telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                strafeLeftEncoder(5, 0.75);
                sleep(100);
            }

            //make sure you are 12 inches from board
            while ((distanceSensor.getDistance(DistanceUnit.INCH) > 8.125 || distanceSensor.getDistance(DistanceUnit.INCH) < 7.875) && opModeIsActive()) {
                if (distanceSensor.getDistance(DistanceUnit.INCH) < 8) {
                    goForwardEncoder(8 - distanceSensor.getDistance(DistanceUnit.INCH), 0.2);
                } else if (distanceSensor.getDistance(DistanceUnit.INCH) > 8) {
                    goBackwardEncoder(distanceSensor.getDistance(DistanceUnit.INCH) - 8, 0.2);
                }
            }

            //telemetry AprilTag
            DESIRED_TAG_ID = 003;
            telemetryAprilTag(DESIRED_TAG_ID);
            sleep(100);
            armEncoderDrive(1250, 0.5);
            liftEncoderDrive(-2000, -1.0);
            clawR.setPosition(clawROpen);

            while(distanceSensor.getDistance(DistanceUnit.INCH) < 24) {
                strafeLeftEncoder(4, 0.2);
                sleep(100);
            }


            strafeLeftEncoder(10, 0.2);
            goBackwardEncoder(20, 0.2);
            break;
        }

    }


    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
        sleep(500);
    }

    protected void baseAngle(double currentAngle) {
        if (currentAngle < 0) {
            rotate(-currentAngle, 0.5);
        } else {
            rotate(currentAngle, 0.5);
        }
    }

    protected double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /*
    -----------------------------------------------------------
     */

    protected void rotate(double degrees, double power) {
        double tgtPower = power;
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        if (degrees > 0) {   // turn left
            frontLeft.setPower(tgtPower);
            backLeft.setPower(tgtPower);
            frontRight.setPower(tgtPower);
            backRight.setPower(tgtPower);
            while (getAngle() < degrees) {
                sleep(1);
            }
        } else if (degrees < 0) {   // turn right
            frontLeft.setPower(-tgtPower);
            backLeft.setPower(-tgtPower);
            frontRight.setPower(-tgtPower);
            backRight.setPower(-tgtPower);
            while (getAngle() > degrees) {
                sleep(1);
            }

        } else return;

        stopMotors();
        sleep(50);
    }

    protected void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    private void armEncoderDrive(int armInches, double speed) {
        arm.setTargetPosition(armInches);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(speed);
    }

    private void liftEncoderDrive(int liftInches, double speed) {
        lift.setTargetPosition(liftInches);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
    }

    protected void encoderDrive(double speed,
                                double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                                double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        final double MINSPEED = 0.30; // Start at this power
        final double SPEEDINCR = 0.2; // And increment by this much each cycle
        double curSpeed; // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //DUE TO ORIENTATION OF MOTORS, LEFT MOTORS HAVE TO HAVE SIGN REVERSED FOR DISTANCES. maybe.
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (-FrontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (-BackLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (BackRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            while (frontLeft.getTargetPosition() != newFrontLeftTarget) {
                frontLeft.setTargetPosition(newFrontLeftTarget);
                sleep(1);
            }
            while (frontRight.getTargetPosition() != newFrontRightTarget) {
                frontRight.setTargetPosition(newFrontRightTarget);
                sleep(1);
            }
            while (backLeft.getTargetPosition() != newBackLeftTarget) {
                backLeft.setTargetPosition(newBackLeftTarget);
                sleep(1);
            }
            while (backRight.getTargetPosition() != newBackRightTarget) {
                backRight.setTargetPosition(newBackRightTarget);
                sleep(1);
            }

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // curSpeed = Math.abs(speed); // Make sure its positive
            curSpeed = 0.3;
            //curSpeed = Math.min(MINSPEED, speed);


            frontLeft.setPower(curSpeed);
            backLeft.setPower(curSpeed);
            frontRight.setPower(curSpeed);
            backRight.setPower(curSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy()&& backRight.isBusy())) { //might need to change this to all being busy..
            }
                // And rewrite the motor speeds
                frontLeft.setPower(curSpeed);
                backLeft.setPower(curSpeed);
                frontRight.setPower(curSpeed);
                backRight.setPower(curSpeed);
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }

    protected void goBackwardEncoder(double goBakInches, double mymotorPower) {
        //frontRight.setPower(tgtPower);
        //frontLeft.setPower(-tgtPower);
        //backRight.setPower(tgtPower);
        //backLeft.setPower(-tgtPower);
        encoderDrive(mymotorPower, -goBakInches, -goBakInches, -goBakInches, -goBakInches, 30);

    }

    protected void goForwardEncoder(double goForInches, double mymotorPower) {
        //frontRight.setPower(-tgtPower);
        //frontLeft.setPower(tgtPower);
        //backRight.setPower(-tgtPower);
        //backLeft.setPower(tgtPower);
        encoderDrive(mymotorPower, goForInches, goForInches, goForInches, goForInches, 30);
    }

    private void strafeRightEncoder(double strafeRightInch, double mymotorPower) {
        //frontRight.setPower(tgtPower);
        //frontLeft.setPower(tgtPower);
        //backRight.setPower(-tgtPower);
        //backLeft.setPower(-tgtPower);
        encoderDrive(mymotorPower, strafeRightInch, -strafeRightInch, -strafeRightInch, strafeRightInch, 30);

    }

    private void strafeLeftEncoder(double strafeLeftInch, double mymotorPower) {
        //frontRight.setPower(-tgtPower);
        //frontLeft.setPower(-tgtPower);
        //backRight.setPower(tgtPower);
        //backLeft.setPower(tgtPower);
        encoderDrive(mymotorPower, -strafeLeftInch, strafeLeftInch, strafeLeftInch, -strafeLeftInch, 30);
    }

    private void sleepAndDrive(int degrees) {
        sleep(25);
        rotate(degrees, 0.75);
    }

//    private void initTfod() {
//
//        tfod = new TfodProcessor.Builder().setModelAssetName(TFOD_MODEL_ASSET).setModelLabels(LABELS).setModelAspectRatio(16.0 / 9.0).build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        tfod.setMinResultConfidence(0.75f);
//    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()
// end method initAprilTag()

    private void telemetryAprilTag(int DESIRED_TAG_ID) {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        while(!targetFound && opModeIsActive()) {
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    sleep(500);
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break; // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);

                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    telemetry.update();
                }
            }
            strafeLeftEncoder(3.125, 0.2);

        }
    }   // end method telemetryAprilTag()


//    private String telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//
//            label = recognition.getLabel();
//        }   // end for() loop
//
//        return label;
//    }   // end method telemetryTfod()

    private void distanceSensorOption() {
        double pos = 0.1; // initial position
        servo.setPosition(pos);
        sleep(100);
        pos1 = false;
        pos2 = false;
        pos3 = false;

        for(int i = 0; i < 5; i++) {
            pos += .01;
            servo.setPosition(pos);
            telemetry.addData("inches:", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(100);
            if(sensor.getDistance(DistanceUnit.INCH) <= 30) {
                pos2 = true;
            }
        }
        for(int i = 0; i < 25; i++) {
            pos += .01;
            servo.setPosition(pos);
            telemetry.addData("inches:", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(100);
            if (sensor.getDistance(DistanceUnit.INCH) <= 30) {
                pos3 = true;
            } else if (!pos2 && !pos3) {
                pos1 = true;
            }
        }

        telemetry.addData("pos1: ", pos1);
        telemetry.addData("pos2: ", pos2);
        telemetry.addData("pos3: ", pos3);
        telemetry.update();
    }
}