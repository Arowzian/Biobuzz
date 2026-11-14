package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.PedroPathing.Procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.PedroPathing.Procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.PedroPathing.Procedures.PinpointTuner;

public class Tuning {
    // Tuners go here

    @Tuner
    public static Procedure mecanumTuner(){
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner(){
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner(){
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }


}