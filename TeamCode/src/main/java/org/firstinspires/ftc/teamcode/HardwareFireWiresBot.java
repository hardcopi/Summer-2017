package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class HardwareFireWiresBot {
    private static final double SHOOTER_WAIT_TIME = 1000;
    private static final double SHOOTER_SHOOT_STRENGTH = .5;
    private static final double SHOOTER_REVERSE_STRENGTH = -.2;
    private static final float SHOOTER_SERVO_UP = -1;
    private static final float SHOOTER_SERVO_DOWN = 1;

    static final double COUNTS_PER_MOTOR_REV = 1220;
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;


    public DcMotor r = null;
    /* DC Motors */
    public DcMotor intakeMotor, leftShooter, rightShooter, leftMotor, rightMotor, strafeMotor = null;
    public ArrayList<DcMotor> motorArray = new ArrayList<DcMotor>(Arrays.asList(
            leftMotor, rightMotor, intakeMotor, leftShooter, rightShooter, strafeMotor));

    /* Servo Motors */
    public Servo shootServo = null;
    public Servo liftServo = null;
    public Servo pusherServo = null;

    /* Sensor Time */
    public OpticalDistanceSensor ods;
    public ColorSensor color;

    public Map<DcMotor, String> m = new HashMap<DcMotor, String>();
    public long start_time;
    public int distance = 0;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareFireWiresBot() {

    }

    public void setStart() {
        start_time = System.currentTimeMillis();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Sensors
        if (hwMap.opticalDistanceSensor.get("ods") != null) {
            ods = hwMap.opticalDistanceSensor.get("ods");
        }
        if (hwMap.colorSensor.get("color") != null) {
            color = hwMap.colorSensor.get("color");
        }

        // Define and Initialize Servos
        if (hwMap.servo.get("shoot-servo") != null) {
            shootServo = hwMap.servo.get("shoot-servo");
        }

        if (hwMap.servo.get("lift-servo") != null) {
            liftServo = hwMap.servo.get("lift-servo");
        }

        if (hwMap.servo.get("pusher-servo") != null) {
            pusherServo = hwMap.servo.get("pusher-servo");
        }

        // Define and Initialize Motors
        if (hwMap.dcMotor.get("left_drive") != null) {
            leftMotor = hwMap.dcMotor.get("left_drive");
            leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        }
        if (hwMap.dcMotor.get("right_drive") != null) {
            rightMotor = hwMap.dcMotor.get("right_drive");
            rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        }
        if (hwMap.dcMotor.get("intake_motor") != null) {
            intakeMotor = hwMap.dcMotor.get("intake_motor");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        if (hwMap.dcMotor.get("strafe_motor") != null) {
            strafeMotor = hwMap.dcMotor.get("strafe_motor");
            strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (hwMap.dcMotor.get("left_shooter_motor") != null) {
            leftShooter = hwMap.dcMotor.get("left_shooter_motor");
            leftShooter.setDirection(DcMotor.Direction.REVERSE);

        }
        if (hwMap.dcMotor.get("right_shooter_motor") != null) {
            rightShooter = hwMap.dcMotor.get("right_shooter_motor");
            rightShooter.setDirection(DcMotor.Direction.FORWARD);
        }

        // Set all motors to zero power
        for (DcMotor motorVar : motorArray) {
            if (motorVar != null) {
                motorVar.setPower(0);
                motorVar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /**
     * Fire Command
     */
    public void fire() {
        long setTime = System.currentTimeMillis();
        leftShooter.setPower(SHOOTER_SHOOT_STRENGTH);
        rightShooter.setPower(SHOOTER_SHOOT_STRENGTH);
    }

    /**
     * Fire Farther Command
     */
    public void fireFarther() {
        long setTime = System.currentTimeMillis();
        leftShooter.setPower((SHOOTER_SHOOT_STRENGTH * 2));
        rightShooter.setPower((SHOOTER_SHOOT_STRENGTH * 2));
    }

    /**
     * Stop Firing
     */
    public void stop_firing() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    /**
     * Set intake power
     *
     * @param power
     */
    public void intake(float power) {
        intakeMotor.setPower(power);
    }

    /**
     * Lift Servo
     *
     * @param distance
     */
    public void move_shoot_servo(float distance) {
        long setTime = System.currentTimeMillis();
        if (distance == -1) {
            leftShooter.setPower(SHOOTER_REVERSE_STRENGTH);
            rightShooter.setPower(SHOOTER_REVERSE_STRENGTH);
            shootServo.setPosition(distance);
        } else {
            shootServo.setPosition(distance);
        }
    }

    public void drive(double left, double right) {
        left = joystick_conditioning((float) left, (float) .2, (float) .05, (float) .1);
        right = joystick_conditioning((float) right, (float) .2, (float) .05, (float) .1);
        leftMotor.setPower((float) -left);
        rightMotor.setPower((float) -right);
    }

    public void strafe(double power) {
        strafeMotor.setPower((float) power);
    }

    /**
     * Condition the joystick
     *
     * @param x
     * @param db   - Deadband
     * @param off  - Offset
     * @param gain - Gain
     * @return
     */
    public float joystick_conditioning(float x, float db, float off, float gain) {
        float output = 0;
        boolean sign = (x > 0);

        x = Math.abs(x);
        if (x > db) {
            output = (float) (off - ((off - 1) * Math.pow(((db - x) / (db - 1)), gain)));
            output *= sign ? 1 : -1;
        }
        return output;
    }
}


