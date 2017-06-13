package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareFireWiresBot;

/**
 * Created by rlester on 6/13/2017.
 */

public class Auto_Methods extends LinearOpMode {
    // Tetrix = 1440
    // Neverest = 1220
    static final double COUNTS_PER_MOTOR_REV = 1220;
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double CONVERSION = 13.4; // Number of inches in 360
    static final double WHITE_THRESHOLD = .2;
    static final double COLOR_THRESHOLD = 2;
    HardwareFireWiresBot robot = new HardwareFireWiresBot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.ods.enableLed(true);
        robot.color.enableLed(false);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }
    /**
     * Turn # of degrees based on encoder
     */
    public void turn(double degrees, double speed) {
        double distance = (CONVERSION / 360) * degrees;
        drive(speed, distance, -distance, 5);
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void drive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /**
     * Shoot
     */
    public void shoot(float number) {
        robot.fire();
        sleep(2000);
        robot.shootServo.setPosition(1);
        sleep(500);
        robot.stop_firing();
        if (number == 2) {
            sleep(500);
            robot.shootServo.setPosition(1);
            robot.fire();
            sleep(2000);
            robot.shootServo.setPosition(1);
            sleep(500);
            robot.stop_firing();
        }
        robot.shootServo.setPosition(0);
    }

    /**
     * Strafe
     */
    public void strafe(double speed, long seconds, String direction) {
        if (direction == "left" || direction == "left_line") {
            speed = -speed;
        }
        if (direction == "left_line" || direction == "right_line") {
            while (robot.ods.getLightDetected() < WHITE_THRESHOLD) {
                robot.strafe(speed);
            }
        } else {
            robot.strafe(speed);
            sleep((seconds * 1000));
        }
        robot.strafe(0);
    }

    /**
     * Bump beacon
     */
    public void bump(double speed, String color) {
        if (color == "blue") {
            while (robot.color.blue() < COLOR_THRESHOLD) {
                drive(speed, 2, 2, 2);
                sleep(1000);
                drive(-speed, 2, 2, 2);
                sleep(500);
                drive(speed, 2, 2, 2);
                telemetry.addData("Blue", robot.color.blue());
                telemetry.addData("Bump", "Beacon Bumped");
                telemetry.update();
            }
        } else {
            while (robot.color.red() < COLOR_THRESHOLD) {
                drive(speed, 2, 2, 2);
                sleep(1000);
                drive(-speed, 2, 2, 2);
                sleep(500);
                drive(speed, 2, 2, 2);
                telemetry.addData("Red", robot.color.red());
                telemetry.addData("Bump", "Beacon Bumped");
                telemetry.update();
            }
        }
    }

}