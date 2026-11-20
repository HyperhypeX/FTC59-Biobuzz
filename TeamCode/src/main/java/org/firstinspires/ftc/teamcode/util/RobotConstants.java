package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotConstants {
    public static class Drivetrain {
        public static String leftFront = "frontLeftMotor";
        public static String leftRear = "backLeftMotor";
        public static String rightFront = "frontRightMotor";
        public static String rightRear = "backRightMotor";
        public static DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD;
    }

    public static class Intake {
        public static String intake = "intakeMotor";
        public static double intakeVelocity = Math.PI*10; // rad/s = 300 RPM
        // Flip this if the intake spins the wrong way.
        public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.REVERSE;
    }

    public static class Outtake {
        public static String outtake = "outtakeMotor";
        public static String feeder = "feederMotor";
        public static double feederVelocity = Math.PI*10; //300 RPM
        public static double outtakeVelocity = 250;
        public static DcMotorSimple.Direction outtakeDirection = DcMotorSimple.Direction.REVERSE;
    }
}
