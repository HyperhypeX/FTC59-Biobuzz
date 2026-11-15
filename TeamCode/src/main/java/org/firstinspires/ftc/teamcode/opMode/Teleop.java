package org.firstinspires.ftc.teamcode.opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name="TeleOpMain", group="TeleOpModes")
public class Teleop extends CommandOpMode {
    private static final double STICK_DEADBAND = 0.15;
    private static final double TRIGGER_THRESHOLD = 0.3;

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    private MecanumDrive drivetrain;
    private Intake intake;
    private Outtake outtake;

    // Shared on/off state so L2 (intake) and R1 (intake + feeder) always agree.
    private boolean intakeOn = false;
    private boolean feederOn = false;
    private boolean slowModeOn = false;
    private boolean outtakeOn = false;

    // Outtake target speed in rad/s. Starts at the RobotConstants value and can be nudged
    // up and down with D-pad right/left while driving. Resets to the constant on each run.
    private static final double OUTTAKE_STEP = 5.0;
    private static final double OUTTAKE_MAX = 628.0; // ~6000 RPM free speed
    private double outtakeTarget = RobotConstants.Outtake.outtakeVelocity;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driver);

        drivetrain = new MecanumDrive();
        configureDrive();

        // The skeleton robot has no intake/outtake in its config, so those motors come
        // back null, and we leave their bindings off rather than NPE on a button press.
        boolean hasIntake = robot.intakeMotor != null;
        // The feeder and the outtake wheel are independent: a robot with a feeder but no
        // shooter motor should still be able to run the feeder.
        boolean hasFeeder = robot.feederMotor != null;
        boolean hasOuttake = robot.outtakeMotor != null;

        if (hasIntake) {
            intake = new Intake();
            configureIntake();
        }
        if (hasFeeder || hasOuttake) {
            outtake = new Outtake();
        }
        if (hasOuttake) {
            configureOuttake();
        }
        if (hasFeeder) {
            configureFeeder();
        }
        if (hasIntake && hasFeeder) {
            configureFeedAndIntake();
            configureReverseIntake();
        }

        telemetry.addData("Drive mode", robot.imu != null ? "field-centric" : "ROBOT-CENTRIC (no imu in config)");
        telemetry.addData("Intake", hasIntake ? "ready" : "not in config - bindings off");
        telemetry.addData("Feeder", hasFeeder ? "ready" : "not in config - bindings off");
        telemetry.addData("Outtake", hasOuttake ? "ready" : "not in config - bindings off");
        // Where the code actually found each motor, so the config can be checked against
        // the real wiring: "<hub info> port N".
        telemetry.addData("intakeMotor", describe(robot.intakeMotor));
        telemetry.addData("feederMotor", describe(robot.feederMotor));
        telemetry.addData("outtakeMotor", describe(robot.outtakeMotor));
        telemetry.update();
    }

    // Live diagnostics: what the code sees from the sticks and what it is commanding.
    @Override
    public void run() {
        super.run();
        telemetry.addData("Left stick x / y", "%.2f / %.2f", driver.getLeftX(), driver.getLeftY());
        telemetry.addData("Right stick x", "%.2f", driver.getRightX());
        telemetry.addData("Power LF / LR / RF / RR", "%.2f / %.2f / %.2f / %.2f",
                robot.leftFront.getPower(), robot.leftRear.getPower(),
                robot.rightFront.getPower(), robot.rightRear.getPower());
        telemetry.addData("Slow mode", drivetrain.getSlowMode());
        if (robot.outtakeMotor != null) {
            telemetry.addData("Outtake velocity set to (rad/s)", "%.0f  [%s]",
                    outtakeTarget, outtakeOn ? "ON" : "off");
            telemetry.addData("Outtake actual (rad/s)", "%.0f", Math.abs(outtake.getOuttakeVelocity()));
        }
        // Raw controller check: whichever of these moves is the one the Driver Station
        // has assigned. TeleOpMain only reads gamepad1. (id -1 = nothing assigned.)
        telemetry.addData("gamepad1 id / left x", "%d / %.2f", gamepad1.getGamepadId(), gamepad1.left_stick_x);
        telemetry.addData("gamepad2 id / left x", "%d / %.2f", gamepad2.getGamepadId(), gamepad2.left_stick_x);
        telemetry.update();
    }

    //Field-centric drive
    private void configureDrive() {
        drivetrain.setDefaultCommand(new RunCommand(() -> {
            // Slow mode is toggled with Share (BACK); R2 is the outtake and D-pad down reverses.
            drivetrain.setSlowMode(slowModeOn);
            drivetrain.drive(
                    deadband(driver.getLeftY()),
                    deadband(driver.getLeftX()),
                    deadband(driver.getRightX()));
        }, drivetrain));
        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> slowModeOn = !slowModeOn));
        if (robot.imu != null) {
            driver.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                    new InstantCommand(() -> robot.imu.resetYaw()));
        }
    }

    private void configureOuttake() {
        // R2 (right trigger): toggles the outtake on each fresh pull; stays on until pulled again.
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> {
                    outtakeOn = !outtakeOn;
                    if (outtakeOn) {
                        outtake.setOuttakeVelocity(outtakeTarget);
                    } else {
                        outtake.stopOuttakeMotor();
                    }
                }, outtake));

        // D-pad right / left: raise / lower the outtake target speed by 5 rad/s. If the
        // outtake is running, the new speed applies immediately.
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> adjustOuttakeTarget(OUTTAKE_STEP)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> adjustOuttakeTarget(-OUTTAKE_STEP)));

        // D-pad up, held: run the outtake wheel backwards (pull balls in / clear a jam).
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    outtakeOn = false;
                    outtake.setOuttakePower(-0.5);
                }, outtake))
                .whenReleased(new InstantCommand(outtake::stopOuttakeMotor, outtake));
    }

    private void adjustOuttakeTarget(double delta) {
        outtakeTarget = Math.max(0, Math.min(OUTTAKE_MAX, outtakeTarget + delta));
        if (outtakeOn) {
            outtake.setOuttakeVelocity(outtakeTarget);
        }
    }

    // L1, held: run the feeder.
    private void configureFeeder() {
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> setFeeder(true)))
                .whenReleased(new InstantCommand(() -> setFeeder(false)));
    }

    // L2 (left trigger): toggles the intake on each fresh pull rather than while held.
    private void configureIntake() {
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> setIntake(!intakeOn)));
    }

    // R1 (right bumper): toggles the feeder and the intake together until pressed again.
    private void configureFeedAndIntake() {
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    boolean start = !(intakeOn && feederOn);
                    setIntake(start);
                    setFeeder(start);
                }));
    }

    // D-pad down, held: run the intake and the feeder backwards together to clear a jam.
    // Releasing it stops both.
    private void configureReverseIntake() {
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> setReverse(true)))
                .whenReleased(new InstantCommand(() -> setReverse(false)));
    }

    private void setReverse(boolean on) {
        intakeOn = on;
        feederOn = on;
        intake.setIntakeMotorVelocity(on ? -RobotConstants.Intake.intakeVelocity : 0);
        if (on) {
            outtake.setFeederVelocity(-RobotConstants.Outtake.feederVelocity);
        } else {
            outtake.stopFeederMotor();
        }
    }

    private void setIntake(boolean on) {
        intakeOn = on;
        intake.setIntakeMotorVelocity(on ? RobotConstants.Intake.intakeVelocity : 0);
    }

    private void setFeeder(boolean on) {
        feederOn = on;
        if (on) {
            outtake.setFeederVelocity(RobotConstants.Outtake.feederVelocity);
        } else {
            outtake.stopFeederMotor();
        }
    }

    //Reverse intake and feeder together to clear a jam.
    // (Might need later but unnecessary for now)
    //private void configureJamClear() {
    //driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
    //.whenPressed(new InstantCommand(() -> {
    //outtake.setFeederPower(-1);
    //intake.setIntakePower(-0.5);
    //}, outtake, intake))
    //.whenReleased(new InstantCommand(() -> {
    //outtake.stopFeederMotor();
    //intake.stopMotor();
    //}, outtake, intake));
    //}

    private static String describe(com.qualcomm.robotcore.hardware.DcMotorEx motor) {
        if (motor == null) return "not in config";
        return motor.getConnectionInfo() + ", port " + motor.getPortNumber();
    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0;
    }
}
