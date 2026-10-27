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

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driver);

        drivetrain = new MecanumDrive();
        configureDrive();

        // The skeleton robot has no intake/outtake in its config, so those motors come
        // back null and we leave their bindings off rather than NPE on a button press.
        boolean hasIntake = robot.intakeMotor != null;
        boolean hasOuttake = robot.outtakeMotor != null && robot.feederMotor != null;

        if (hasIntake) {
            intake = new Intake();
            configureIntake();
        }
        if (hasOuttake) {
            outtake = new Outtake();
            configureOuttake();
        }

        telemetry.addData("Drive mode", robot.imu != null ? "field-centric" : "ROBOT-CENTRIC (no imu in config)");
        telemetry.addData("Intake", hasIntake ? "ready" : "not in config - bindings off");
        telemetry.addData("Outtake", hasOuttake ? "ready" : "not in config - bindings off");
        telemetry.update();
    }

    //Field-centric drive
    private void configureDrive() {
        drivetrain.setDefaultCommand(new RunCommand(() -> {
            drivetrain.setSlowMode(
                    driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD);
            drivetrain.drive(
                    deadband(driver.getLeftY()),
                    deadband(driver.getLeftX()),
                    deadband(driver.getRightX()));
        }, drivetrain));

        if (robot.imu != null) {
            driver.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                    new InstantCommand(() -> robot.imu.resetYaw()));
        }
    }

    private void configureOuttake() {
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> outtake.setOuttakePower(-0.5), outtake))
                .whenReleased(new InstantCommand(outtake::stopOuttakeMotor, outtake));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(
                        () -> outtake.setOuttakeVelocity(RobotConstants.Outtake.outtakeVelocity), outtake))
                .whenReleased(new InstantCommand(outtake::stopOuttakeMotor, outtake));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(
                        () -> outtake.setFeederVelocity(RobotConstants.Outtake.feederVelocity), outtake))
                .whenReleased(new InstantCommand(outtake::stopFeederMotor, outtake));
    }

    private void configureIntake() {
        // Left trigger toggles the intake on each fresh pull rather than while held.
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> intake.setIntakeMotorVelocity(
                        intake.getIntakePower() > 0.1 ? 0 : RobotConstants.Intake.intakeVelocity), intake));
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

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0;
    }
}
