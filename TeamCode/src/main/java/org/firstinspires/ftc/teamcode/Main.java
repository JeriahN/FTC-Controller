package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RevHub Controller", group="TeleOp")
public class Main extends OpMode {

    // Declare motor objects
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    @Override
    public void init() {
        // Initialize hardware mappings for the motors connected to ports 0, 1, 2, and 3
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        // Set motor directions (reverse right side to make tank drive work properly)
        motor0.setDirection(DcMotor.Direction.FORWARD);  // Left front motor
        motor1.setDirection(DcMotor.Direction.FORWARD);  // Left back motor
        motor2.setDirection(DcMotor.Direction.REVERSE);  // Right front motor
        motor3.setDirection(DcMotor.Direction.REVERSE);  // Right back motor

        // Set motors to run without encoders
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Controller input for tank drive
        double leftStick = gamepad1.left_stick_x;
        double rightStick = gamepad1.right_stick_x;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

//        Controls:
//        Left Stick: Left, Right, Forward, Backward
//        Right Stick: Rotate Left, Rotate Right
//        Left Trigger: Move Backward
//        Right Trigger: Move Forward
//        Left Bumper: Strafe Left
//        Right Bumper: Strafe Right

//      To Strafe the robot left, the bottom left motor and the top right motor should move forward and the top left motor and the bottom right motor should move backward.
//      To Strafe the robot right, the bottom left motor and the top right motor should move backward and the top left motor and the bottom right motor should move forward.

//        To rotate the robot left, the top left motor and the bottom left motor should move backward and the top right motor and the bottom right motor should move forward.
//        To rotate the robot right, the top left motor and the bottom left motor should move forward and the top right motor and the bottom right motor should move backward.

//        To Strafe the robot south west, the top left and bottom right motors should move backward
//        To Strafe the robot south east, the top right and bottom left motors should move backward

        // Telemetry data send input values to the driver station
        telemetry.addData("Status", "Running");
        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.addData("Left Trigger", leftTrigger);
        telemetry.addData("Right Trigger", rightTrigger);
        telemetry.addData("Left Bumper", leftBumper);
        telemetry.addData("Right Bumper", rightBumper);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when the op mode ends
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
