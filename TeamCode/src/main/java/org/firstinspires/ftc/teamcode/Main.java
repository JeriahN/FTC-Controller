package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "RevHub Controller", group = "TeleOp")
public class Main extends OpMode {
    // Motor and Servo Declarations
    private DcMotorEx Frm, Blm, Brm, Flm;
    private DcMotorEx RobotLiftLeft, RobotLiftRight;
    private DcMotorEx ArmLift;
    private Servo ArmLiftBucketDumper;
    private Servo armActuator;
    private CRServo armActuatorIntake;
    private Servo intakeDumper;

    // Target Variables
    private double tgtArmLift = 0;
    private double tgtArmActuator = 0;

    // Controller Input Variables
    double leftStickX, rightStickX, leftStickY, rightStickY;
    double leftTrigger, rightTrigger;
    boolean leftBumper, rightBumper, dPadUp, dPadDown, dPadLeft, dPadRight, a, b, x, y;

    // Dead Zones and Scaling
    private double leftStickDeadZone = 0.05;
    private double rightStickDeadZone = 0.05;
    private double rightTriggerDeadZone = 0.15;
    private double inputSensitivity = 0.6; // Exponential scaling factor for smoother input
    private boolean actuatorBucketActive = false;
    private boolean dPadRightActiveLast = false;

    // Arm Actuator Variables
    private final double actMin = 0;
    private final double actMax = 55;
    private final double ALDumperMin = 0.23;
    private final double ALDumperMax = 1.0;

    @Override
    public void init() {
        // Initialize hardware mappings
        Frm = hardwareMap.get(DcMotorEx.class, "Frm");
        Blm = hardwareMap.get(DcMotorEx.class, "Blm");
        Brm = hardwareMap.get(DcMotorEx.class, "Brm");
        Flm = hardwareMap.get(DcMotorEx.class, "Flm");

        RobotLiftLeft = hardwareMap.get(DcMotorEx.class, "RobotLiftLeft");
        RobotLiftRight = hardwareMap.get(DcMotorEx.class, "RobotLiftRight");
        ArmLift = hardwareMap.get(DcMotorEx.class, "ArmLift");
        ArmLiftBucketDumper = hardwareMap.get(Servo.class, "ALBDumper");

        armActuator = hardwareMap.get(Servo.class, "ArmActuator");
        armActuatorIntake = hardwareMap.get(CRServo.class, "ArmActuatorIntake");
        intakeDumper = hardwareMap.get(Servo.class, "intakeDumper");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Capture and process controller inputs
        leftStickX = scaleInput(gamepad1.left_stick_x * -1);
        rightStickX = scaleInput(gamepad1.right_stick_x * -1);
        leftStickY = scaleInput(gamepad1.left_stick_y * -1);
        rightStickY = scaleInput(gamepad1.right_stick_y * -1);

        leftTrigger = gamepad2.left_trigger;
        rightTrigger = gamepad2.right_trigger;

        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;

        dPadUp = gamepad2.dpad_up;
        dPadDown = gamepad2.dpad_down;
        dPadLeft = gamepad2.dpad_left;
        dPadRight = gamepad2.dpad_right;

        a = gamepad2.a;
        b = gamepad2.b;
        x = gamepad2.x;
        y = gamepad2.y;

        // Robot Lift Controls
        if (a) {
            tgtArmLift = 1;
        } else if (b) {
            tgtArmLift = -1;
        } else {
            tgtArmLift = 0;
        }

        // Arm Lift Controls
        if (x) {
            RobotLiftLeft.setPower(1);
            RobotLiftRight.setPower(1);
        } else if (y) {
            RobotLiftLeft.setPower(-1);
            RobotLiftRight.setPower(-1);
        } else {
            RobotLiftLeft.setPower(0);
            RobotLiftRight.setPower(0);
        }

        // Arm Actuator Controls
        if (rightTrigger >= rightTriggerDeadZone) {
            tgtArmActuator = rightTrigger;
        } else {
            tgtArmActuator = 0;
        }
        armActuator.setPosition(calculateArmActuatorServoPos(tgtArmActuator));

        // Toggle Bucket Dumper
        if (dPadRight && !dPadRightActiveLast) {
            actuatorBucketActive = !actuatorBucketActive;
        }
        dPadRightActiveLast = dPadRight;
        intakeDumper.setPosition(actuatorBucketActive ? ALDumperMax : ALDumperMin);

        // Intake Controls
        double targetIntakeSpeed = 0;
        if (dPadDown) {
            targetIntakeSpeed = leftTrigger;
        } else {
            targetIntakeSpeed = -leftTrigger;
        }
        armActuatorIntake.setPower(targetIntakeSpeed);

        if (dPadUp) {
            ArmLiftBucketDumper.setPosition(1);
        } else {
            ArmLiftBucketDumper.setPosition(0);
        }

        // Drive Controls
        double frmSpeed = rightStickY;
        double brmSpeed = rightStickY;
        double flmSpeed = leftStickY;
        double blmSpeed = leftStickY;

        frmSpeed = smoothPowerChange(Frm.getPower(), frmSpeed, 0.05); // Adjust the rate as needed
        blmSpeed = smoothPowerChange(Blm.getPower(), blmSpeed, 0.05);
        brmSpeed = smoothPowerChange(Brm.getPower(), brmSpeed, 0.05);
        flmSpeed = smoothPowerChange(Flm.getPower(), flmSpeed, 0.05);

        if (leftBumper) {
            frmSpeed = -1;
            blmSpeed = 1;
        } else if (rightBumper) {
            frmSpeed = 1;
            blmSpeed = -1;
        }

        Frm.setPower(frmSpeed);
        Blm.setPower(blmSpeed);
        Brm.setPower(brmSpeed);
        Flm.setPower(flmSpeed);

        ArmLift.setPower(tgtArmLift);

        telemetry.update();
    }

    @Override
    public void stop() {
        Frm.setPower(0);
        Blm.setPower(0);
        Brm.setPower(0);
        Flm.setPower(0);
    }

    // Helper Functions
    public double calculateArmActuatorServoPos(double targetPosition) {
        return actMin + (targetPosition / (actMax - actMin)) * (actMax - actMin);
    }

    public double scaleInput(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), inputSensitivity);
    }

    private double smoothPowerChange(double currentPower, double targetPower, double rate) {
        if (Math.abs(targetPower - currentPower) < rate) {
            return targetPower;
        }
        return currentPower + Math.signum(targetPower - currentPower) * rate;
    }
}
