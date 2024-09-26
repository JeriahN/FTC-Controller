package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RevHub Controller", group="TeleOp")
public class Main extends OpMode {

    // Declare motor objects
//    Motor 0 is Top Left
    private DcMotor Frm;
//    Motor 1 is Bottom Left
    private DcMotor Blm;
//    Motor 2 is Bottom Right
    private DcMotor Brm;
//    Motor 3 is Top Right
    private DcMotor Flm;

    // Declare servo objects
    private Servo servoTest;

    double tgtPower = 0;

    // Controller input for tank drive
    double leftStickX;
    double rightStickX;
    double leftStickY;
    double rightStickY;
    double leftTrigger;
    double rightTrigger;
    boolean leftBumper;
    boolean rightBumper;
    boolean dPadUp;
    boolean dPadDown;
    boolean dPadLeft;
    boolean dPadRight;

//    Controller Variables
    double leftStickDeadZone = 0.05;

    @Override
    public void init() {
        // Initialize hardware mappings for the motors connected to ports 0, 1, 2, and 3
        Frm = hardwareMap.get(DcMotor.class, "Frm");
        Blm = hardwareMap.get(DcMotor.class, "Blm");
        Brm = hardwareMap.get(DcMotor.class, "Brm");
        Flm = hardwareMap.get(DcMotor.class, "Flm");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Set motor directions (reverse right side to make tank drive work properly)
        Frm.setDirection(DcMotor.Direction.FORWARD);  // Left front motor
        Blm.setDirection(DcMotor.Direction.FORWARD);  // Left back motor
        Brm.setDirection(DcMotor.Direction.REVERSE);  // Right front motor
        Flm.setDirection(DcMotor.Direction.REVERSE);  // Right back motor

        // Set motors to run without encoders
        Frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // To Strafe the robot left, the bottom left motor and the top right motor should move forward and the top left motor and the bottom right motor should move backward.
        // To Strafe the robot right, the bottom left motor and the top right motor should move backward and the top left motor and the bottom right motor should move forward.

        // To rotate the robot left, the top left motor and the bottom left motor should move backward and the top right motor and the bottom right motor should move forward.
        // To rotate the robot right, the top left motor and the bottom left motor should move forward and the top right motor and the bottom right motor should move backward.

        // To Strafe the robot south west, the top left and bottom right motors should move backward
        // To Strafe the robot south east, the top right and bottom left motors should move backward

        // Controller Input on floating values range from -1 to 1 for joy sticks and from 0 to 1 for triggers
        // Controller speed is in a range for motor
        // Controls:
        // Right Trigger: Forward
        // Left Trigger: Backward
        // Right Bumper: Rotate Right
        // Left Bumper: Rotate Left
        // Right Stick X: Rotate Right
        // Left Stick X: Strafe Left, Strafe Right
        // Left Stick Y: Forward, Backward
        // Right Stick X: Rotate Left, Rotate Right
        // D-Pad Up: Tank Drive Forward
        // D-Pad Down: Tank Drive Backward
        // D-Pad Left: Strafe Left
        // D-Pad Right: Strafe Right

        // Controller input for tank drive
        leftStickX = gamepad1.left_stick_x * -1;
        rightStickX = gamepad1.right_stick_x * -1;
        leftStickY = gamepad1.left_stick_y * -1;
        rightStickY = gamepad1.right_stick_y * -1;
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;
        dPadLeft = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;

        // Telemetry data send input values to the driver station
        telemetry.addData("Status", "Running");
        telemetry.addData("Left Stick Horizontal", leftStickX);
        telemetry.addData("Left Stick Vertical", leftStickY);
        telemetry.addData("Right Stick Horizontal", rightStickX);
        telemetry.addData("Right Stick Vertical", rightStickY);
        telemetry.addData("Left Trigger", leftTrigger);
        telemetry.addData("Right Trigger", rightTrigger);
        telemetry.addData("Left Bumper", leftBumper);
        telemetry.addData("Right Bumper", rightBumper);
        telemetry.addData("D-Pad Up", dPadUp);
        telemetry.addData("D-Pad Down", dPadDown);
        telemetry.addData("D-Pad Left", dPadLeft);
        telemetry.addData("D-Pad Right", dPadRight);
        telemetry.update();

//        Speed and Control Variables
        double frmSpeed = 0;
        double blmSpeed = 0;
        double brmSpeed = 0;
        double flmSpeed = 0;

        tgtPower = -this.gamepad1.left_stick_y;

//        Right Trigger: Forward
        if (rightTrigger > 0) {
            frmSpeed = rightTrigger;
            blmSpeed = rightTrigger;
            brmSpeed = rightTrigger;
            flmSpeed = rightTrigger;
        }

//        Left Trigger: Backward
        if (leftTrigger > 0) {
            frmSpeed = -leftTrigger;
            blmSpeed = -leftTrigger;
            brmSpeed = -leftTrigger;
            flmSpeed = -leftTrigger;
        }

//        Right Bumper: Rotate Right
        if (rightBumper) {
            frmSpeed = 1;
            blmSpeed = 1;
            brmSpeed = -1;
            flmSpeed = -1;
        }

//        Left Bumper: Rotate Left
        if (leftBumper) {
            frmSpeed = -1;
            blmSpeed = -1;
            brmSpeed = 1;
            flmSpeed = 1;
        }

//        Left Stick X: Strafe Left, Strafe Right
        if (leftStickX > leftStickDeadZone) {
            frmSpeed = -leftStickX;
            blmSpeed = leftStickX;
            brmSpeed = -leftStickX;
            flmSpeed = leftStickX;
        } else if (leftStickX < -leftStickDeadZone) {
            frmSpeed = -leftStickX;
            blmSpeed = leftStickX;
            brmSpeed = -leftStickX;
            flmSpeed = leftStickX;
        }

//        Left Stick Y: Forward, Backward
        if (leftStickY > leftStickDeadZone) {
            frmSpeed = leftStickY;
            blmSpeed = leftStickY;
            brmSpeed = leftStickY;
            flmSpeed = leftStickY;
        } else if (leftStickY < -leftStickDeadZone) {
            frmSpeed = leftStickY;
            blmSpeed = leftStickY;
            brmSpeed = leftStickY;
            flmSpeed = leftStickY;
        }

//        Right Stick X: Rotate Left, Rotate Right
        if (rightStickX > leftStickDeadZone) {
            frmSpeed = rightStickX;
            blmSpeed = rightStickX;
            brmSpeed = -rightStickX;
            flmSpeed = -rightStickX;
        } else if (rightStickX < -leftStickDeadZone) {
            frmSpeed = rightStickX;
            blmSpeed = rightStickX;
            brmSpeed = -rightStickX;
            flmSpeed = -rightStickX;
        }

//        D-Pad Up: Tank Drive Forward
        if (dPadUp) {
            frmSpeed = 1;
            blmSpeed = 1;
            brmSpeed = 1;
            flmSpeed = 1;
        }

//        D-Pad Down: Tank Drive Backward
        if (dPadDown) {
            frmSpeed = -1;
            blmSpeed = -1;
            brmSpeed = -1;
            flmSpeed = -1;
        }

//        D-Pad Left: Strafe Left
        if (dPadLeft) {
            frmSpeed = -1;
            blmSpeed = 1;
            brmSpeed = -1;
            flmSpeed = 1;
        }

//        D-Pad Right: Strafe Right
        if (dPadRight) {
            frmSpeed = 1;
            blmSpeed = -1;
            brmSpeed = 1;
            flmSpeed = -1;
        }

        // check to see if we need to move the servo.
        if(gamepad1.y) {
            // move to 0 degrees.
            servoTest.setPosition(0);
        } else if (gamepad1.x || gamepad1.b) {
            // move to 90 degrees.
            servoTest.setPosition(0.5);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            servoTest.setPosition(1);
        }

        // Set motor power based on controller input
        Frm.setPower(frmSpeed);
        Blm.setPower(blmSpeed * -1);
        Brm.setPower(brmSpeed * -1);
        Flm.setPower(flmSpeed);
    }

        public void stop() {
        // Stop all motors when the op mode ends
        Frm.setPower(0);
        Blm.setPower(0);
        Brm.setPower(0);
        Flm.setPower(0);
        servoTest.setPosition(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
