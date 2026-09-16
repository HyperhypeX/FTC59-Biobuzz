package pedroPathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import subsystems.Intake;
import subsystems.Outtake;
import util.RobotConstants;

@Autonomous(name = "AutonBlue", group = "Test")
public class AutonBlue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Poses
    private final Pose startPoseBottom = new Pose(12, 60, Math.toRadians(90));
    private final Pose startPoseTop = new Pose(120,120, Math.toRadians(315));
    private final Pose row1Control = new Pose(48, 72, Math.toRadians(270));
    private final Pose row1Start = new Pose(36, 96, Math.toRadians(270));
    private final Pose row1End = new Pose(36, 120, Math.toRadians(270));
    private final Pose row2Control = new Pose(48, 72, Math.toRadians(270));
    private final Pose row2Start = new Pose(60, 96, Math.toRadians(270));
    private final Pose row2End = new Pose(60, 120, Math.toRadians(270));
    private final Pose row3Control = new Pose(72, 72, Math.toRadians(270));
    private final Pose row3Start = new Pose(84, 96, Math.toRadians(270));
    private final Pose row3End = new Pose(84, 120, Math.toRadians(270));
    private final Pose scorePoseTop = new Pose(84, 60, Math.toRadians(135));
    private final Pose scorePoseBottom = new Pose(12, 84, Math.toRadians(90));

    // Paths
    private Path row1, row2, row3, pickup1, pickup2, pickup3, score, score1, score2, score3;

    private Intake intake;
    private Outtake outtake;

    public void buildPaths() {
//        score = line(startPoseTop, scorePoseTop)
//                .heading(Interpolator.linear(startPoseTop.heading(), scorePoseTop.heading()));
        row3 = curve(scorePoseBottom, row3Control, row3Start)
                .heading(Interpolator.linear(scorePoseBottom.heading(), row3Start.heading()));
        pickup3 = path(
                line(row3Start, row3End)
                        .heading(Interpolator.linear(row3Start.heading(), row3End.heading())),
                line(row3End, row3Start)
                        .heading(Interpolator.linear(row3End.heading(), row3Start.heading())));
        score3 = curve(row3Start, row3Control, scorePoseBottom)
                .heading(Interpolator.linear(row3Start.heading(), scorePoseBottom.heading()));

        row2 = curve(scorePoseBottom, row2Control, row2Start)
                .heading(Interpolator.linear(scorePoseBottom.heading(), row2Start.heading()));
        pickup2 = path(
                line(row2Start, row2End)
                        .heading(Interpolator.linear(row2Start.heading(), row2End.heading())),
                line(row2End, row2Start)
                        .heading(Interpolator.linear(row2End.heading(), row2Start.heading())));
        score2 = curve(row2Start, row2Control, scorePoseBottom)
                .heading(Interpolator.linear(row2Start.heading(), scorePoseBottom.heading()));

        row1 = curve(scorePoseBottom, row1Control, row1Start)
                .heading(Interpolator.linear(scorePoseBottom.heading(), row1Start.heading()));
        pickup1 = path(
                line(row1Start, row1End)
                        .heading(Interpolator.linear(row1Start.heading(), row1End.heading())),
                line(row1End, row1Start)
                        .heading(Interpolator.linear(row1End.heading(), row1Start.heading())));
        score1 = curve(row1Start, row1Control, scorePoseBottom)
                .heading(Interpolator.linear(row1Start.heading(), scorePoseBottom.heading()));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
//                follower.follow(score);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.follow(row1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.follow(pickup1);
                    setPathState(3);
                }
            case 3:
                if (follower.completion() < 0.75) {
                    intake.setIntakeMotorVelocity(RobotConstants.Intake.intakeVelocity);
                } else {
                    intake.stopMotor();
                }
                if(!follower.isBusy()) {
                    intake.stopMotor();
                    follower.follow(score1);
                    setPathState(4);
                }
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                outtake.setFeederVelocity(RobotConstants.Outtake.feederVelocity);
//                outtake.setOuttakeVelocity(RobotConstants.Outtake.outtakeVelocity);
//                if(!follower.isBusy() && pathTimer.milliseconds() > 2000) {
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    outtake.stopMotors();
//                    follower.follow(row2);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    intake.setIntakeMotorVelocity(RobotConstants.Intake.intakeVelocity);
//                    follower.follow(pickup2);
//                    setPathState(6);
//                }
//            case 6:
//                if (follower.completion() < 0.75) {
//                    intake.setIntakeMotorVelocity(RobotConstants.Intake.intakeVelocity);
//                } else {
//                    intake.stopMotor();
//                }
//                if(!follower.isBusy()) {
//                    intake.stopMotor();
//                    follower.follow(score2);
//                    setPathState(7);
//                }
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                outtake.setFeederVelocity(RobotConstants.Outtake.feederVelocity);
//                outtake.setOuttakeVelocity(RobotConstants.Outtake.outtakeVelocity);
//                if(!follower.isBusy() && pathTimer.milliseconds() > 2000) {
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    outtake.stopMotors();
//                    follower.follow(row3);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    intake.setIntakeMotorVelocity(RobotConstants.Intake.intakeVelocity);
//                    follower.follow(pickup3);
//                    setPathState(9);
//                }
//            case 9:
//                if (follower.completion() < 0.75) {
//                    intake.setIntakeMotorVelocity(RobotConstants.Intake.intakeVelocity);
//                } else {
//                    intake.stopMotor();
//                }
//                if(!follower.isBusy()) {
//                    intake.stopMotor();
//                    follower.follow(score3);
//                    setPathState(10);
//                }
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.reset();


        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        buildPaths();
        follower.setPose(startPoseTop);

        intake = new Intake();
        outtake = new Outtake();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
