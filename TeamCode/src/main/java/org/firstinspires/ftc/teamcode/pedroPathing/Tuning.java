package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.TwoWheelTuner;

/**
 * Registers the tuning procedures so they appear on the Driver Station.
 * <p>
 * TunerScanner finds every {@code @Tuner} method in TeamCode by reflection, so each one
 * must be static and take no parameters or the scan throws at startup.
 */
public class Tuning {
    // How this robot is built, handed to the tuners that need to construct hardware.
    private static final Function<HardwareMap, Localizer> LOCALIZER =
            h -> new PinpointLocalizer(h, Constants.localizerConfig);
    private static final Function<HardwareMap, Drivetrain> DRIVETRAIN =
            h -> new Mecanum(h, Constants.drivetrainConfig);
    private static final Supplier<Algorithm> ALGORITHM =
            () -> new Foresight(Constants.algorithmConfig);

    // ******************* THIS ROBOT ******************* //

    @Tuner(name = "Mecanum")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight")
    public static Procedure foresight() {
        return new ForesightTuner(LOCALIZER, DRIVETRAIN);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(DRIVETRAIN, LOCALIZER, ALGORITHM);
    }

    // ******************* OTHER LOCALIZERS ******************* //
    // Kept so swapping odometry hardware only means picking a different entry.
    @Tuner(name = "Two Wheel")
    public static Procedure twoWheel() {
        return new TwoWheelTuner();
    }

    @Tuner(name = "Three Wheel")
    public static Procedure threeWheel() {
        return new ThreeWheelTuner();
    }

    @Tuner(name = "Three Wheel + IMU")
    public static Procedure threeWheelImu() {
        return new ThreeWheelIMUTuner();
    }

    @Tuner(name = "OTOS")
    public static Procedure otos() {
        return new OTOSTuner();
    }

    @Tuner(name = "OctoQuad")
    public static Procedure octoQuad() {
        return new OctoQuadTuner();
    }
}
