package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto 1", group = "FireBot")
public class Auto_1 extends Auto_Methods {
    @Override
    public void runOpMode() {

//        drive(DRIVE_SPEED, 12, 12, 5.0);
        turn(180, .2);
        shoot(2);
//        strafe(.5, 1, "left");
//        bump(.3, "blue");

        telemetry.addData("Status", "Auto Complete");
        telemetry.update();
    }
}
