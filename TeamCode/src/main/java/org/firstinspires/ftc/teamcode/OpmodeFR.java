package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.button.Button;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "Main", name = "Teleop1")
public class OpmodeFR extends OpMode {

    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;
    private IMU imu;

    private double maxSpeed = 0.3;

    @Override
    public void init() {
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(RevOrientation));

    }

    public void driver(double forward, double rotate, double strafe) {

        double FLmotorPower = forward + rotate + strafe;
        double BLmotorPower = forward - strafe + rotate;
        double FRmotorPower = forward - strafe - rotate;
        double BRmotorPower = forward + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(FLmotorPower), Math.abs(BLmotorPower)),
                Math.max(Math.abs(FRmotorPower), Math.abs(BRmotorPower))
        ));

        FLmotor.setPower(maxSpeed * (FLmotorPower / maxPower));
        FRmotor.setPower(maxSpeed * (FRmotorPower / maxPower));
        BLmotor.setPower(maxSpeed * (BLmotorPower / maxPower));
        BRmotor.setPower(maxSpeed * (BRmotorPower / maxPower));
    }


    public void driveFieldRelative(double forward, double strafe, double rotate){

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe  = r * Math.cos(theta);

        driver(newForward, newStrafe, rotate);

    }

    @Override
    public void loop() {

        if (gamepad1.circleWasPressed()) {
            maxSpeed = maxSpeed + 0.1;
            if (maxSpeed > 1.0) {
                maxSpeed = 1.0;
                System.out.println(maxSpeed);
            }
            if (gamepad1.crossWasPressed()) {
                maxSpeed = maxSpeed - 0.1;
                if (maxSpeed < 0.0) {
                    maxSpeed = 0.0;
                    System.out.println(maxSpeed);
                }

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                driveFieldRelative(forward, strafe, rotate);

                telemetry.update();


            }
        }
    }}