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

    public GamepadEx driver;

    // Limelight
    public Limelight3A limelight;

    // Color Sensor
//    public ColorSensor colorSensor;

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

        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE
//        rightRear.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE
        leftRear.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE

        // TODO: LogoFacingDirection/UsbFacingDirection must match how the Control Hub is
        // actually bolted on, or field-centric drive will be rotated by a fixed offset.
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            ));
            imu.initialize(parameters);
            imu.resetYaw();
        }

        // Not on the skeleton robot yet, so these use tryGet and stay null when the
        // config has no such device. get() would throw and kill the OpMode at init.
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

        // ******************* LIMELIGHT ******************* //
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.pipelineSwitch(0);
//        limelight.start();

        // ******************* COLOR SENSOR ******************* //
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }


}