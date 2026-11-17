package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotConstants {
    public static class Drivetrain {
        public static String leftFront = "frontLeftMotor";
        public static String leftRear = "backLeftMotor";
        public static String rightFront = "frontRightMotor";
        public static String rightRear = "backRightMotor";

        // Single source of truth: RobotHardware applies these for TeleOp and
        // pedroPathing.Constants hands the same values to Pedro for auton. Change them
        // here only, or the robot will drive differently in TeleOp than in auton.
        // TODO: Verify on blocks - these have not been confirmed on the real robot.
        public static DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD;
    }

    // Settings for the simple timed auto (opMode/AutoLeave). Tune these on the field.
    public static class Auto {
        // Motor power while driving, 0..1. Start low until you have seen it move.
        public static double drivePower = 0.4;
        // How long to drive away from the start, then the same time back.
        public static double driveOutSeconds = 1;
        // Battery voltage compensation: a weak battery makes the same power move the robot
        // less, so the drive power is scaled by nominalVoltage / actualVoltage (limited to
        // 0.8..1.5x). Set nominalVoltage to what a fully charged battery reads (~12.8).
        // Set voltageCompensation to false to turn it off.
        public static boolean voltageCompensation = true;
        public static double nominalVoltage = 12.8;

        // Sideways trim while driving, 0..~0.3. Positive moves the robot to ITS right,
        // negative to its left. Use these when the robot slides off line: it ends up too
        // far left -> add a small positive value. Out and back are separate because a
        // lopsided robot does not always drift the same way in both directions.
        public static double strafeOutPower = 0.0;
        public static double strafeBackPower = 0.0;

        // Time spent driving back. It starts equal to the way out; raise it a little if the
        // robot ends up short of where it started, lower it if it overshoots.
        public static double returnSeconds = 0;
        // Pause at the far point before coming back.
        public static double pauseSeconds = 0.5;

        // Heading hold (uses the Control Hub IMU): nudges the wheels to keep the robot
        // pointing the way it started, which stops it veering when one side is heavier.
        // Set headingHold to false to go back to plain open-loop driving.
        public static boolean headingHold = true;
        // Turn power added per radian of heading error (1 rad is about 57 degrees).
        public static double headingKp = 1.0;
        // Largest turn correction allowed, so a bad reading can't spin the robot.
        public static double maxCorrection = 0.2;
        // Set too false to only leave and stay out (no return).
        public static boolean returnToStart = true;
    }

    public static class Intake {
        public static String intake = "intakeMotor";
        public static double intakeVelocity = Math.PI*2;
        // Flip this if the intake spins the wrong way.
        public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.REVERSE;
    }

    public static class Outtake {
        public static String outtake = "outtakeMotor";
        public static String feeder = "feederMotor";
        public static double feederVelocity = 120;
        // rad/s. goBILDA Yellow Jacket, ~1150 RPM free speed = ~120 rad/s; start near 70%
        // so it can hold speed under load. Tune from the Driver Station readout.
        public static double outtakeVelocity = 400.0;
        // Positive velocity (R2) must push balls OUT of the outtake. Flip this if it pulls them in.
        public static DcMotorSimple.Direction outtakeDirection = DcMotorSimple.Direction.REVERSE;
    }
}
