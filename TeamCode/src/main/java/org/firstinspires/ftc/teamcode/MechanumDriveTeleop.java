package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDriveTeleop {

    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;
    private IMU imu;

    public void init(HardwareMap HWmap) {
        FLmotor = HWmap.get(DcMotor.class, "FLmotor");
        BLmotor = HWmap.get(DcMotor.class, "BLmotor");
        FRmotor = HWmap.get(DcMotor.class, "FRmotor");
        BRmotor = HWmap.get(DcMotor.class, "BRmotor");

        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);

        imu = HWmap.get(IMU.class, "imu");
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


        double maxPower = 1.0;
        double maxSpeed = 0.3;
        maxPower = Math.max(maxPower, Math.abs(FLmotorPower));
        maxPower = Math.max(maxPower, Math.abs(BLmotorPower));
        maxPower = Math.max(maxPower, Math.abs(FRmotorPower));
        maxPower = Math.max(maxPower, Math.abs(BRmotorPower));

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

}
