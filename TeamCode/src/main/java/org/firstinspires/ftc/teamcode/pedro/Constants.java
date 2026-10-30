package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLmotor");
        c.frontRightName.set("FRmotor");
        c.backLeftName.set("BLmotor");
        c.backRightName.set("BRmotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(1.0162481074481073);
        c.angularScalar.set(0.990587783617251);
        c.offset.set(new Pose(0.18622931532972442, -0.34842904158464566));
        c.linearUnit.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23523571646916328);
                Controller secondaryTranslationalForward = Controller.proportional(0.08691331162124796);
                Controller primaryTranslationalLateral = Controller.proportional(0.2859550697959422);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10565275742941196);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01431785443810079));
                c.brake.set(Controller.proportionalFeedforward(0.012170176272385671));

                c.headingFeedback.set(Controller.proportional(3.5394049725618917));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.5101518726982393, -0.15679184101629762));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04698018744499275, 0.03596389314034849));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021730669249583477, 0.0024627677033068943));

                c.maxAchievableForwardVelocity.set(58.164658231482214);
                c.maxAchievableStrafeVelocity.set(45.80811125945409);
                c.naturalForwardDeceleration.set(76.1222383535369);
                c.naturalStrafeDeceleration.set(102.82144408012859);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}