
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main TeleOp")
public class MainOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        MechanumDriveTeleop drive = new MechanumDriveTeleop();
        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            drive.driveFieldRelative(forward, strafe, rotate);
        }
    }
}
