package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOp 2024", group = "Iterative Opmode")
public class TeleOp2024 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Timer timer = new Timer();
    BNO055IMU imu;
    private boolean isSpeedToggle = false;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotorEx launcher;


    private Servo clawL;
    private Servo clawR;
    private Servo armServo;
    private Servo clawRotation;
    private DcMotorEx lift;
    private DcMotorEx arm;
    private DigitalChannel pSensorBack;
    private DigitalChannel pSensorFront;
    private DistanceSensor distanceSensor;
    private Servo servo;
    private DistanceSensor sensor;

    Orientation lastAngles = new Orientation();
    private double globalAngle;

    private double motorPower = 0.75;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP... maybe 1??
    static final double WHEEL_DIAMETER_INCHES = 10 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (12 / 9.6)/* (12/10.25) */ * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing ...");
        telemetry.update();

        servo = hardwareMap.get(Servo.class, "servo");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");

        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        armServo = hardwareMap.get(Servo.class, "armServo");

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        telemetry.addData("lift.getCurrentPosition", lift.getCurrentPosition());
        telemetry.addData("arm.getCurrentPosition(): ", arm.getCurrentPosition());
        telemetry.update();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);

        armServo.setPosition(0.25);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("lift.currentPosition(): ", lift.getCurrentPosition());
        telemetry.addData("arm.currentPosition(): ", arm.getCurrentPosition());
        telemetry.addData("armServo.getPosition(): ", armServo.getPosition());
        telemetry.addData("clawR.getPosition(): ", clawR.getPosition());
        telemetry.addData("runtime.milliseconds(): ", runtime.milliseconds());

        telemetry.addData("motorPower: ", motorPower);

        telemetry.update();

        //toggle speed
        if (gamepad1.right_bumper) {
            if (!isSpeedToggle) {
                motorPower = 0.875;
                isSpeedToggle = true;
            } else {
                motorPower = 0.75;
                isSpeedToggle = false;
            }
        } else if (gamepad1.left_bumper) {
            if (!isSpeedToggle) {
                motorPower = 0.625;
                isSpeedToggle = true;
            } else {
                motorPower = 0.75;
                isSpeedToggle = false;
            }
        }


        if (gamepad1.left_stick_x < -0.85) {
            strafeRight(motorPower);
        }
        else if (gamepad1.left_stick_x > 0.85) {
            strafeLeft(motorPower);
        }
        else {
            if (Math.abs(this.gamepad1.right_stick_x) > 0.3) {
                frontRight.setPower(motorPower * 0.775 * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                frontLeft.setPower(motorPower * 0.775 * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                backRight.setPower(motorPower * 0.775 * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
                backLeft.setPower(motorPower * 0.775 * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
            }
            else {
                frontRight.setPower(motorPower * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                frontLeft.setPower(motorPower * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
            }
        }



        //claw
        if(gamepad1.a) {
            clawL.setPosition(0.85);
            clawR.setPosition(0.0000000001);
        } else if(gamepad1.b) {
            clawL.setPosition(0.375);
            clawR.setPosition(0.625);
        } else if(gamepad1.x) {
            resetAngle();
            stopMotors();
        }

        //hang from bar
        if(gamepad1.y) {
            armEncoderDrive(888, 0.5);
        }

        //armServo
        if(gamepad1.dpad_up) {
            armServo.setPosition(armServo.getPosition() + 0.4);
        } else if(gamepad1.dpad_down) {
            armServo.setPosition(armServo.getPosition() - 0.4);
        }

        //arm
        if(gamepad2.right_stick_y < -0.2) {
            arm.setTargetPosition(arm.getCurrentPosition() + 75);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(0.42 * Math.abs(gamepad2.right_stick_y));
        } else if(gamepad2.right_stick_y > 0.2) {
            arm.setTargetPosition(arm.getCurrentPosition() - 75);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(0.42 * -Math.abs(gamepad2.right_stick_y));
        }

        //lift
        if (gamepad2.left_stick_y < -0.2) { //going up?
            lift.setTargetPosition(lift.getCurrentPosition() + 150);
            lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift.setPower(10 * -Math.abs(gamepad2.left_stick_y));
        } else if (gamepad2.left_stick_y > 0.2) {
            lift.setTargetPosition(lift.getCurrentPosition() - 150);
            lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift.setPower(10 * Math.abs(gamepad2.left_stick_y));                 // start to check lift down -> is go home ?
        }

        //gamepad2 buttons, arm + lift
        if(gamepad2.a) {
            armEncoderDrive(-265, -0.75);
        } else if(gamepad2.y) {
            armEncoderDrive(1250, 0.5);
            encoderDrive(-2000, -1.0);
            armServo.setPosition(armServo.getPosition() + 0.4);
        } else if(gamepad2.b) {
            runtime.reset();
            encoderDrive(7500, 0.5);
            while(runtime.milliseconds() <= 3000) {
                telemetry.update();
            }
            armEncoderDrive(-600, -0.5);
        } else if(gamepad2.x) {
            encoderDrive(-600, -0.5);
            armEncoderDrive(-265, -0.5);
        }


        //airplane launcher
        if(gamepad1.back) {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    launcher.setPower(0);
                }
            }, 500);
            launcher.setPower(-0.99);
        } else if(gamepad2.left_bumper && gamepad2.right_bumper) {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    launcher.setPower(0);
                }
            }, 500);
            launcher.setPower(-0.99);
        }
    }

    // -------------------------------------------------------------------------------------------------------------

    private void encoderDrive(int liftInches, double speed) {
        lift.setTargetPosition(liftInches);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while(lift.getCurrentPosition() >= liftInches) {
            lift.setPower(speed);
        }
    }

    private void armEncoderDrive(int armInches, double speed) {
        arm.setTargetPosition(armInches);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while(arm.getCurrentPosition() >= armInches) {
            arm.setPower(speed);
        }
    }

    protected void drive(double speed,
                         double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                         double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        final double MINSPEED = 0.30; // Start at this power
        final double SPEEDINCR = 0.2; // And increment by this much each cycle
        double curSpeed; // Keep track of speed as we ramp

        //DUE TO ORIENTATION OF MOTORS, LEFT MOTORS HAVE TO HAVE SIGN REVERSED FOR DISTANCES. maybe.
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (FrontLeftInches * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (-FrontRightInches * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (BackLeftInches * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (-BackRightInches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        while (frontLeft.getTargetPosition() != newFrontLeftTarget) {
            frontLeft.setTargetPosition(newFrontLeftTarget);

        }
        while (frontRight.getTargetPosition() != newFrontRightTarget) {
            frontRight.setTargetPosition(newFrontRightTarget);

        }
        while (backLeft.getTargetPosition() != newBackLeftTarget) {
            backLeft.setTargetPosition(newBackLeftTarget);

        }
        while (backRight.getTargetPosition() != newBackRightTarget) {
            backRight.setTargetPosition(newBackRightTarget);

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
        while ((runtime.seconds() < timeoutS) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy()&& backRight.isBusy())) { //might need to change this to all being busy..
        }
        // And rewrite the motor speeds
        frontLeft.setPower(curSpeed);
        backLeft.setPower(curSpeed);
        frontRight.setPower(curSpeed);
        backRight.setPower(curSpeed);

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
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    protected void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}