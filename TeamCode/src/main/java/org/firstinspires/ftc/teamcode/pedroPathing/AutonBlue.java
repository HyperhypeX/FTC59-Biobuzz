package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

// @Autonomous is off on purpose: buildPaths() is still empty, so row1/pickup1/score1 are
// null and the first case would NPE. Re-enable once the paths are actually built.
//@Autonomous(name = "AutonBlue", group = "Autonomous")
public class AutonBlue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose autonRed1 = poseFactory.of(56, 8, 90);
    private final Pose moveToShoot = poseFactory.of(56, 36, 90);
    private final Pose loadingWall = poseFactory.of(5, 36, 270);
    private final Pose point3 = poseFactory.of(5, 5, 270);
    private final Pose loadingSpot = poseFactory.of(55.6777, 36, 90);
    private final Pose returnPose = poseFactory.of(60, 3, 90);
    //Paths
    private Path row1, row2, row3, pickup1, pickup2, pickup3, score, score1, score2, score3;

    private Intake intake;
    private Outtake outtake;

    public void buildPaths() {

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


        follower = Constants.create(hardwareMap);
        buildPaths();
        follower.setPose(autonRed1);

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
