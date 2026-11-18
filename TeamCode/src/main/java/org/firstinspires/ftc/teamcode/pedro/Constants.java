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

    private static double kU_heading = 3.5394049725618917;
    private static double tU_heading
            = 0.5;
    private static double kP_heading = 0.6*kU_heading;
    private static double kI_heading = 2*kP_heading/tU_heading;
    private static double kD_heading = kP_heading * tU_heading / 8;

    private static double kU_p_forward = 0.23523571646916328;
    private static double tU_p_forward = 0.5;
    private static double kP_p_forward = 0.6*kU_p_forward;
    private static double kI_p_forward = 2*kP_p_forward/tU_p_forward;
    private static double kD_p_forward = kP_p_forward * tU_p_forward / 8;

    private static double kU_s_forward = 0.08691331162124796;
    private static double tU_s_forward = 0.5;
    private static double kP_s_forward = 0.6*kU_s_forward;
    private static double kI_s_forward = 2*kP_s_forward/tU_s_forward;
    private static double kD_s_forward = kP_s_forward * tU_s_forward / 8;

    private static double kU_p_lat = 0.2859550697959422;
    private static double tU_p_lat = 0.5;
    private static double kP_p_lat = 0.6*kU_p_lat;
    private static double kI_p_lat = 2*kP_p_lat/tU_p_lat;
    private static double kD_p_lat = kP_p_lat * tU_p_lat / 8;

    private static double kU_s_lat = 0.10565275742941196;
    private static double tU_s_lat = 0.5;
    private static double kP_s_lat = 0.6*kU_s_lat;
    private static double kI_s_lat = 2*kP_s_lat/tU_s_lat;
    private static double kD_s_lat = kP_s_lat * tU_s_lat / 8;
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

                Controller primaryTranslationalForward =
                        Controller.pid(kU_p_forward, kI_p_forward, kD_p_forward);

                Controller secondaryTranslationalForward =
                        Controller.pid(kP_s_forward, kI_s_forward, kD_s_forward);

                Controller primaryTranslationalLateral =
                        Controller.pid(kP_p_lat, kI_p_lat, kD_p_lat);

                Controller secondaryTranslationalLateral =
                        Controller.pid(kP_s_lat, kI_s_lat, kD_s_lat);

                Controller headingController =
                        Controller.pid(kP_heading, kI_heading, kD_heading);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01431785443810079));
                c.brake.set(Controller.proportionalFeedforward(0.012170176272385671));

                c.headingFeedback.set(headingController);
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.5101518726982393, -0.15679184101629762));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04698018744499275, 0.03596389314034849));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021730669249583477, 0.0024627677033068943));

                c.maxAchievableForwardVelocity.set(58.164658231482214);
                c.maxAchievableStrafeVelocity.set(45.80811125945409);
                c.naturalForwardDeceleration.set(76.1222383535369);
                c.naturalStrafeDeceleration.set(102.82144408012859);

                c.maxPathSpeed.set(0.6);
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