package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

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

@TeleOp(name = "Real Test", group = "Iterative Opmode")
public class test extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private Servo clawL;
    private Servo clawR;
    private DcMotorEx lift;
    private DigitalChannel pSensor;

    private double motorPower = 0.75;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing ...");
        telemetry.update();


        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        lift = hardwareMap.get(DcMotorEx.class, "lift");


        telemetry.addData("lift current position", lift.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("lift.currentPosition(): ", lift.getCurrentPosition());
        telemetry.addData("motorPower: ", motorPower);
        telemetry.addData("frontLeft: ", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight: ", frontRight.getCurrentPosition());
        telemetry.addData("backLeft: ", backLeft.getCurrentPosition());
        telemetry.addData("backRight: ", backRight.getCurrentPosition());

        telemetry.update();

        if(gamepad1.a) { //frontLeft
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + 100);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setPower(0.75);
            telemetry.addData("frontLeft: ", frontLeft.getCurrentPosition());
            telemetry.update();
        } else if(gamepad1.b) { //frontRight
            frontRight.setTargetPosition(frontLeft.getCurrentPosition() + 100);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setPower(0.75);
            telemetry.addData("frontRight: ", frontRight.getCurrentPosition());
            telemetry.update();
        } else if(gamepad1.x) { //backLeft
            backLeft.setTargetPosition(frontLeft.getCurrentPosition() + 100);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setPower(0.75);
            telemetry.addData("backLeft: ", backLeft.getCurrentPosition());
            telemetry.update();
        } else if(gamepad1.y) { //backRight
            backRight.setTargetPosition(frontLeft.getCurrentPosition() + 100);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setPower(0.75);
            telemetry.addData("backRight: ", backRight.getCurrentPosition());
            telemetry.update();
        }




        if (gamepad1.left_stick_x < -0.85) {
            strafeRight(motorPower);
        }
        else if (gamepad1.left_stick_x > 0.85) {
            strafeLeft(motorPower);
        }
        else {
            if (Math.abs(this.gamepad1.right_stick_x) > 0.3) {
                frontRight.setPower(motorPower * 0.775 * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                frontLeft.setPower(motorPower * 0.775 * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                backRight.setPower(motorPower * 0.775 * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
                backLeft.setPower(motorPower * 0.775 * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
            }
            else {
                frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backRight.setPower(motorPower * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backLeft.setPower(motorPower * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
            }
        }

        if (gamepad1.dpad_up) {
            frontLeft.setPower(-motorPower * 0.3);
            frontRight.setPower(motorPower * 0.3);
            backLeft.setPower(-motorPower * 0.3);
            backRight.setPower(motorPower * 0.3);
        }
        else if (gamepad1.dpad_down) {
            frontLeft.setPower(motorPower * 0.3);
            frontRight.setPower(-motorPower * 0.3);
            backLeft.setPower(motorPower * 0.3);
            backRight.setPower(-motorPower * 0.3);
        }
        else if (gamepad1.dpad_left) {
            strafeLeft(motorPower * 0.3);
        }
        else if (gamepad1.dpad_right) {
            strafeRight(motorPower * 0.3);
        }

        if (gamepad2.left_stick_y < -0.2) {
            if (lift.getCurrentPosition() > -6000) lift.setTargetPosition(lift.getCurrentPosition() - 100);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.75 * Math.abs(gamepad2.left_stick_y));
        } else if (gamepad2.left_stick_y > 0.2) {
            lift.setTargetPosition(lift.getCurrentPosition() + 100);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.75 * Math.abs(gamepad2.left_stick_y));                 // start to check lift down -> is go home ?
        } else {
            lift.setPower(0);
        }
    }


    // -------------------------------------------------------------------------------------------------------------

    private void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }



    protected void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


}