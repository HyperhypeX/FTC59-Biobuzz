package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

/**
 * Simplest possible auto: drive straight out of the starting square, pause, then drive
 * straight back. It is open-loop (timed), so it needs no odometry or path tuning, and it
 * works even if the Pinpoint is not in the hardware config.
 *
 * Distances are set by time and power in RobotConstants.Auto. Wheel slip and battery
 * voltage change how far it really goes, so check it on the field and adjust.
 *
 * While driving it holds the heading it started with (IMU), so an uneven weight
 * distribution does not pull it off line.
 */
@Autonomous(name = "AutoLeave", group = "Autonomous")
public class AutoLeave extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private double targetYaw = 0;
    private double powerScale = 1.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (RobotConstants.Auto.voltageCompensation && voltage > 1) {
            powerScale = clamp(RobotConstants.Auto.nominalVoltage / voltage, 0.8, 1.5);
        }
        telemetry.addData("Battery", "%.2f V (power x%.2f)", voltage, powerScale);

        telemetry.addData("Plan", "out %.1fs, back %.1fs @ %.2f power%s",
                RobotConstants.Auto.driveOutSeconds,
                RobotConstants.Auto.returnSeconds,
                RobotConstants.Auto.drivePower,
                RobotConstants.Auto.returnToStart ? "" : " (stay out)");
        telemetry.addData("Heading hold", headingHoldAvailable() ? "on" : "off");
        telemetry.addLine("Make sure the robot has clear space ahead of it.");
        telemetry.update();

        // While waiting for START, show the heading. Turn the robot by hand to check it:
        // turning the nose to the LEFT should make this number go UP.
        while (!isStarted() && !isStopRequested()) {
            if (robot.imu != null) {
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentYaw()));
                telemetry.update();
            }
        }
        if (isStopRequested()) return;

        // Whatever way the robot is facing at the start is "straight".
        targetYaw = robot.imu != null ? currentYaw() : 0;

        // Out of the starting square.
        drive(RobotConstants.Auto.drivePower, RobotConstants.Auto.strafeOutPower,
                RobotConstants.Auto.driveOutSeconds, "Leaving");

        if (RobotConstants.Auto.returnToStart) {
            pause(RobotConstants.Auto.pauseSeconds);
            drive(-RobotConstants.Auto.drivePower, RobotConstants.Auto.strafeBackPower,
                    RobotConstants.Auto.returnSeconds, "Returning");
        }

        stopDrive();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    private boolean headingHoldAvailable() {
        return RobotConstants.Auto.headingHold && robot.imu != null;
    }

    private double currentYaw() {
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private void drive(double power, double strafe, double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double turn = 0;
            if (headingHoldAvailable()) {
                // Yaw goes up when the robot turns left. A positive turn command turns the
                // robot right (left wheels forward, right wheels back), so a heading that
                // has drifted left (error > 0) is corrected with a positive turn.
                double error = AngleUnit.normalizeRadians(currentYaw() - targetYaw);
                turn = clamp(RobotConstants.Auto.headingKp * error,
                        -RobotConstants.Auto.maxCorrection, RobotConstants.Auto.maxCorrection);
                telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(error));
                telemetry.addData("Turn correction", "%.2f", turn);
            }
            setDrivePower(power * powerScale, strafe, turn);
            telemetry.addData("Status", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.update();
        }
        stopDrive();
    }

    private void pause(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Status", "Paused");
            telemetry.update();
        }
    }

    // The motor directions are applied in RobotHardware, so equal power on all four
    // wheels drives straight forward (negative = backward). A positive turn adds power to
    // the left wheels and removes it from the right wheels.
    private void setDrivePower(double power, double strafe, double turn) {
        // Same mecanum mix as TeleOp: strafe is positive to the robot's right.
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        robot.leftFront.setPower((power + strafe + turn) / denominator);
        robot.leftRear.setPower((power - strafe + turn) / denominator);
        robot.rightFront.setPower((power - strafe - turn) / denominator);
        robot.rightRear.setPower((power + strafe - turn) / denominator);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
