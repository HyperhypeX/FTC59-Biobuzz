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

@Autonomous(name = "AutoPath", group = "Autonomous")

public class AutonRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose autonRed1 = poseFactory.of(56, 8, 90);
    private final Pose moveToShoot = poseFactory.of(56, 36, 90);
    private final Pose loadingWall = poseFactory.of(5, 36, 270);
    private final Pose point3 = poseFactory.of(5, 5, 270);
    private final Pose loadingSpot = poseFactory.of(55.6777, 36, 90);
    private final Pose returnPose = poseFactory.of(60, 3, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                //Case 1
                follow(follower, line(autonRed1, moveToShoot).linear(autonRed1, moveToShoot)),
                follow(follower, line(moveToShoot, loadingWall).linear(moveToShoot, loadingWall)),
                follow(follower, line(loadingWall, point3).linear(loadingWall, point3)),
                follow(follower, line(point3, loadingSpot).linear(point3, loadingSpot)),
                follow(follower, line(loadingSpot, returnPose).linear(loadingSpot, returnPose)),
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
        follower.setPose(autonRed1);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
