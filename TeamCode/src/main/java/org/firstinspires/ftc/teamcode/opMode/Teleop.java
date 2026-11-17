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

    private boolean outtakeOn = false;
    private double outtakeTarget = RobotConstants.Outtake.outtakeVelocity;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driver);

        drivetrain = new MecanumDrive();
        configureDrive();

        // The skeleton robot has no intake/outtake in its config, so those motors come
            intake = new Intake();
        }
            outtake = new Outtake();
        }

        telemetry.addData("Drive mode", robot.imu != null ? "field-centric" : "ROBOT-CENTRIC (no imu in config)");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        }
        telemetry.update();
    }

    // Field-centric drive
    private void configureDrive() {
                deadband(driver.getLeftY()),
                deadband(driver.getLeftX()),
        if (robot.imu != null) {
            driver.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                    new InstantCommand(() -> robot.imu.resetYaw()));
        }
    }

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
                .whenActive(new InstantCommand(() -> {
                    outtakeOn = !outtakeOn;

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
    }

        outtakeTarget = Math.max(0, Math.min(OUTTAKE_MAX, outtakeTarget + delta));
        if (outtakeOn) {
            outtake.setOuttakeVelocity(outtakeTarget);
        } else {
        }
    }

        }
        }
    }

    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0;
    }
}
