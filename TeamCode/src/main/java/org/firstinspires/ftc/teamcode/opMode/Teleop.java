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

/**
 * Controls (PS4 names):
 *   Left stick / right stick X ... drive / turn (field-centric when an IMU is configured)
 *   Options (Start) .............. reset field-centric heading
 *   R2 ........................... toggle outtake at the set velocity
 *   L2 ........................... toggle intake
 *   L1 ........................... toggle feeder
 *   R1 ........................... toggle intake + feeder together
 *   D-pad up ..................... toggle intake + feeder in reverse (jam clear)
 *   D-pad right / left ........... outtake velocity +10 / -10 rad/s
 *   D-pad down ................... unused
 */
@TeleOp(name="TeleOpMain", group="TeleOpModes")
public class Teleop extends CommandOpMode {
    private static final double STICK_DEADBAND = 0.15;
    private static final double TRIGGER_THRESHOLD = 0.3;
    private static final double OUTTAKE_STEP = 10;      // rad/s per D-pad press
    private static final double OUTTAKE_MAX = 628;      // ~6000 RPM free speed in rad/s

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    private MecanumDrive drivetrain;
    private Intake intake;
    private Outtake outtake;

    // Toggle state. Intake and feeder: 1 = forward, -1 = reverse, 0 = off.
    private boolean outtakeOn = false;
    private double outtakeTarget = RobotConstants.Outtake.outtakeVelocity;
    private int intakeDir = 0;
    private int feederDir = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driver);

        drivetrain = new MecanumDrive();
        configureDrive();

        // The skeleton robot has no intake/outtake in its config, so those motors come
        // back null. Everything below is null-safe: missing mechanisms are simply skipped.
        if (robot.intakeMotor != null) {
            intake = new Intake();
        }
        if (robot.outtakeMotor != null && robot.feederMotor != null) {
            outtake = new Outtake();
        }
        configureMechanisms();

        telemetry.addData("Drive mode", robot.imu != null ? "field-centric" : "ROBOT-CENTRIC (no imu in config)");
        telemetry.addData("Intake", intake != null ? "ready" : "not in config - controls off");
        telemetry.addData("Outtake", outtake != null ? "ready" : "not in config - controls off");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Outtake", outtakeOn ? "ON" : "off");
        telemetry.addData("Outtake velocity set to (rad/s)", outtakeTarget);
        if (outtake != null) {
            telemetry.addData("Outtake actual (rad/s)", outtake.getOuttakeVelocity());
        }
        telemetry.addData("Intake", dirLabel(intakeDir));
        telemetry.addData("Feeder", dirLabel(feederDir));
        telemetry.update();
    }

    // Field-centric drive
    private void configureDrive() {
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
                deadband(driver.getLeftY()),
                deadband(driver.getLeftX()),
                deadband(driver.getRightX())), drivetrain));
        if (robot.imu != null) {
            driver.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                    new InstantCommand(() -> robot.imu.resetYaw()));
        }
    }

    private void configureMechanisms() {
        // R2: toggle outtake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> {
                    outtakeOn = !outtakeOn;
                    applyOuttake();
                }));

        // L2: toggle intake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> {
                    intakeDir = intakeDir == 1 ? 0 : 1;
                    applyIntakeAndFeeder();
                }));

        // L1: toggle feeder
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    feederDir = feederDir == 1 ? 0 : 1;
                    applyIntakeAndFeeder();
                }));

        // R1: toggle intake + feeder together
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    boolean bothForward = intakeDir == 1 && feederDir == 1;
                    intakeDir = feederDir = bothForward ? 0 : 1;
                    applyIntakeAndFeeder();
                }));

        // D-pad up: toggle intake + feeder in reverse
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    boolean bothReverse = intakeDir == -1 && feederDir == -1;
                    intakeDir = feederDir = bothReverse ? 0 : -1;
                    applyIntakeAndFeeder();
                }));

        // D-pad right / left: outtake velocity trim
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> adjustOuttake(OUTTAKE_STEP)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> adjustOuttake(-OUTTAKE_STEP)));
    }

    private void adjustOuttake(double delta) {
        outtakeTarget = Math.max(0, Math.min(OUTTAKE_MAX, outtakeTarget + delta));
        applyOuttake(); // takes effect immediately if the outtake is already running
    }

    private void applyOuttake() {
        if (outtake == null) return;
        if (outtakeOn) {
            outtake.setOuttakeVelocity(outtakeTarget);
        } else {
            outtake.stopOuttakeMotor();
        }
    }

    private void applyIntakeAndFeeder() {
        if (intake != null) {
            intake.setIntakeMotorVelocity(intakeDir * RobotConstants.Intake.intakeVelocity);
        }
        if (outtake != null) {
            outtake.setFeederVelocity(feederDir * RobotConstants.Outtake.feederVelocity);
        }
    }

    private static String dirLabel(int dir) {
        return dir > 0 ? "forward" : dir < 0 ? "REVERSE" : "off";
    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0;
    }
}
