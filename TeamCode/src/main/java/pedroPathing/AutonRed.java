package pedroPathing;

import static com.pedropathing.api.Paths.*;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;

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
                follow(follower, moveToShoot()),
                follow(follower, loadingWall()),
                follow(follower, path3()),
                follow(follower, loadingSpot()),
                follow(follower, returnPose()),
                hold(follower)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
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

    public Path moveToShoot() {
        return line(autonRed1, moveToShoot).linear(autonRed1, moveToShoot);
    }

    public Path loadingWall() {
        return line(moveToShoot, loadingWall).linear(moveToShoot, loadingWall);
    }

    public Path path3() {
        return line(loadingWall, point3).linear(loadingWall, point3);
    }

    public Path loadingSpot() {
        return line(point3, loadingSpot).linear(point3, loadingSpot);
    }

    public Path returnPose() {
        return line(loadingSpot, returnPose).linear(loadingSpot, returnPose);
    }
}
