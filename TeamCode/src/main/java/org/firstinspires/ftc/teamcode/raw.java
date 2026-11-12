
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Raw OTOS Test")
public class raw extends LinearOpMode {

    @Override
    public void runOpMode() {
        SparkFunOTOS otos =
                hardwareMap.get(SparkFunOTOS.class, "otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        boolean calibrated = otos.calibrateImu();
        otos.resetTracking();

        telemetry.addData("Calibration successful", calibrated);
        telemetry.addLine("Keep robot stationary");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D p = otos.getPosition();

            telemetry.addData("Raw X", p.x);
            telemetry.addData("Raw Y", p.y);
            telemetry.addData("Raw heading", p.h);
            telemetry.update();
        }
    }
}
