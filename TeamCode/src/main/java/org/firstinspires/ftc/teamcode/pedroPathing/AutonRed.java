package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.*;

import com.bylazar.field.Line;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRed", group = "Autonomous")

public class AutonRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses from the Pedro Visualizer export. Heading is in degrees.
    private final Pose shootPos = poseFactory.of(56.0258, 2, 90);
    private final Pose flower = poseFactory.of(10, 46.3454, 180);
    private final Pose flowerControl = poseFactory.of(45.3203, 42.5094, 0);
    private final Pose flowerToShootControl = poseFactory.of(29, 40, 0);
    private final Pose shootPos2 = poseFactory.of(56.9676, 0.8788, 90);
    private final Pose secondShoot = poseFactory.of(57.74, 132.97, 270);
    private final Pose secondShootControl = poseFactory.of(93.96, 59.0467, 0);

    // Autonomous routine. Paths only for now: no intake/outtake/feeder actions yet.
    public Command autoRoutine() {
        return sequential(
                // shootPos -> flower
                follow(follower, curve(shootPos, flowerControl, flower).linear(shootPos, flower)),
                // flower -> back to the shoot position
                follow(follower, curve(flower, flowerToShootControl, shootPos2).linear(flower, shootPos2)),
                // shoot position -> second shoot spot
                follow(follower, curve(shootPos2, secondShootControl, secondShoot).linear(shootPos2, secondShoot)),
                hold(follower)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        try {
            follower = Constants.create(hardwareMap);
        } catch (RuntimeException e) {
            // Usually a hardware-config mismatch (e.g. no device named "pinpoint").
            // Show it on the Driver Station instead of crashing the OpMode.
            telemetry.addLine("Follower init FAILED - check the hardware config:");
            telemetry.addLine(String.valueOf(e.getMessage()));
            telemetry.update();
            while (!isStopRequested()) {
                sleep(50);
            }
            return;
        }
        follower.setPose(shootPos);
        follower.update();

        // Before Play: keep reading the Pinpoint so the pose shows on the Driver Station.
        // Nudge the robot by hand and watch x/y to check the Pinpoint tracks correctly.
        // (Nothing is scheduled yet, so the follower is idle and the robot will not move.)
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            telemetry.addLine("Ready - place robot at start pose, then press Play");
            addPoseTelemetry();
            telemetry.update();
        }
        if (isStopRequested()) return;
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            addPoseTelemetry();

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    private void addPoseTelemetry() {
        telemetry.addData("x (in)", "%.2f", follower.pose().x());
        telemetry.addData("y (in)", "%.2f", follower.pose().y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
    }
}
