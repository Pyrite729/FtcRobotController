package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure otosTuner(){
        return new OTOSTuner();}
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(0.9757411488862836);
        c.angularScalar.set(0.9523809523809548);
        c.offset.set(new Pose(0.2763402743602362, 0.25831808255413385));
        c.linearUnit.set(DistanceUnit.INCH);
    });
}
