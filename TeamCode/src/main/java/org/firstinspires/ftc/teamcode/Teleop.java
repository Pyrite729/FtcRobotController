package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;


@TeleOp
public class Teleop extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    private double motor_speed = 0.5;

    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "FLmotor");
        fR = new Motor(hardwareMap, "FRmotor");
        bL = new Motor(hardwareMap,"BLmotor");
        bR = new Motor(hardwareMap,"BRmotor");

        bL.setInverted(true);
        bR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                driverOp.getLeftX() *motor_speed,
                -driverOp.getLeftY() *motor_speed,
                driverOp.getRightX() *motor_speed
        );
        if (gamepad1.circleWasPressed()){
            motor_speed = motor_speed + 0.1;
            if (motor_speed > 1.0){
                motor_speed = 1.0;
            }
            System.out.println("Speed is"+ motor_speed);
        }
        if (gamepad1.crossWasPressed()){
            motor_speed = motor_speed - 0.1;
            if (motor_speed < 0.1){
                motor_speed = 0.1;
            }
        System.out.println("Speed is"+ motor_speed);
    }

}}