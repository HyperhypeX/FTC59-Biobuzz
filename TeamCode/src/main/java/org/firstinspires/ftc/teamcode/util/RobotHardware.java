package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class RobotHardware {
    // Drivetrain
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    // Hardware
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;

    // Intake
    public DcMotorEx intakeMotor;

    // Outtake
    public DcMotorEx outtakeMotor;
    public DcMotorEx feederMotor;
    public Servo outtakeServo, stopServo;


    public GamepadEx driver;

    // Limelight
    public Limelight3A limelight;
    public TelemetryManager telemetryManager;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, GamepadEx driver) {
        this.driver = driver;
        init(hardwareMap);
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // ******************* DRIVETRAIN ******************* //
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);

        // Directions live in RobotConstants so auton and TeleOp cannot disagree.
        leftFront.setDirection(RobotConstants.Drivetrain.leftFrontDirection);
        leftRear.setDirection(RobotConstants.Drivetrain.leftRearDirection);
        rightFront.setDirection(RobotConstants.Drivetrain.rightFrontDirection);
        rightRear.setDirection(RobotConstants.Drivetrain.rightRearDirection);

        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ));
            imu.initialize(parameters);
            imu.resetYaw();
        }

        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.Intake.intake);

        // ******************* OUTTAKE ******************* //
        outtakeMotor = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.Outtake.outtake);
        if (outtakeMotor != null) {
            outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        feederMotor = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.Outtake.feeder);
        if (feederMotor != null) {
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}