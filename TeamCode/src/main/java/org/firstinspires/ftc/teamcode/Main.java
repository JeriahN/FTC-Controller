package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RevHub Controller", group = "TeleOp")
public class Main extends OpMode {
    //    Hardware Declarations
    private DcMotorEx Frm, Blm, Brm, Flm;
    private Servo clawRotater, claw;

    double joystickleftX, joysticklefty, joystickrightx, joystickrighty;
    boolean dPadUp, dPadDown, dPadLeft, dPadRight;
    boolean bButton, xButton, aButton, yButton;
    boolean bButtonLast, xButtonLast, aButtonLast, yButtonLast;
    boolean clawRotaterActive = false, clawActive = false;
    double frmPower, blmPower, brmPower, flmPower;

    final double clawRotatePosMin = 0, clawRotatePosMax = 0.4;
    final double clawPosMin = 0, clawPosMax = 0.4;


    public void init() {
        // Initialize motors and servos
        Frm = hardwareMap.get(DcMotorEx.class, "Frm");
        Blm = hardwareMap.get(DcMotorEx.class, "Blm");
        Brm = hardwareMap.get(DcMotorEx.class, "Brm");
        Flm = hardwareMap.get(DcMotorEx.class, "Flm");

        clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions
        Frm.setDirection(DcMotorSimple.Direction.FORWARD);
        Blm.setDirection(DcMotorSimple.Direction.FORWARD);
        Brm.setDirection(DcMotorSimple.Direction.FORWARD);
        Flm.setDirection(DcMotorSimple.Direction.FORWARD);

        joystickleftX = gamepad1.left_stick_x;
        joysticklefty = gamepad1.left_stick_y;
        joystickrightx = gamepad1.right_stick_x;
        joystickrighty = gamepad1.right_stick_y;

        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;
        dPadLeft = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;

        bButton = gamepad1.b;
        xButton = gamepad1.x;
        aButton = gamepad1.a;
        yButton = gamepad1.y;

        frmPower = joystickrighty;
        blmPower = joystickrighty;
        brmPower = joysticklefty;
        flmPower = joysticklefty;
    }

    public void loop() {
        // Initialize joysticks
        joystickleftX = gamepad1.left_stick_x;
        joysticklefty = gamepad1.left_stick_y;
        joystickrightx = gamepad1.right_stick_x;
        joystickrighty = gamepad1.right_stick_y;

        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;
        dPadLeft = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;

        bButton = gamepad1.b;
        xButton = gamepad1.x;
        aButton = gamepad1.a;
        yButton = gamepad1.y;

        joysticklefty = gamepad1.left_stick_y;
        joystickleftX = gamepad1.left_stick_x;
        joystickrighty = gamepad1.right_stick_y;
        joystickrightx = gamepad1.right_stick_x;

//            Tank Drive Controls
        frmPower = joystickrighty;
        blmPower = joystickrighty;
        brmPower = joysticklefty;
        flmPower = joysticklefty;

        // Set motor powers
        Frm.setPower(frmPower);
        Blm.setPower(blmPower);
        Brm.setPower(brmPower);
        Flm.setPower(flmPower);

        // Control claw rotater with button B
        if (bButton) {
            clawRotater.setPosition(clawRotatePosMax);
            telemetry.addData("Claw Rotater Position", clawRotater.getPosition());
        } else if (xButton) {
            clawRotater.setPosition(0);
            telemetry.addData("Claw Rotater Position", clawRotater.getPosition());
        }

        else {
            //clawRotater.setPosition(clawRotater.getPosition());
            telemetry.addData("Claw Rotater Position", clawRotater.getPosition());
        }

        // Control claw with button A
        if (aButton) {
            claw.setPosition(90);
            telemetry.addData("Claw Position", claw.getPosition());
        } else if (yButton) {
            claw.setPosition(0);
            telemetry.addData("Claw Position", claw.getPosition());
        } else {
            claw.setPosition(claw.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
        }

        if (bButton != bButtonLast) {
            if (bButton) {
                clawRotaterActive = !clawRotaterActive;
                telemetry.addData("Claw Rotater Active", clawRotaterActive);
            }
            bButtonLast = bButton;
            telemetry.addData("Claw Rotater Active", clawRotaterActive);
        }

        



//        Strafe using omni wheels
        if (dPadLeft) {
//            Strafe Left
            Flm.setPower(0.5);
            Brm.setPower(0.5);
            Frm.setPower(-0.5);
            Blm.setPower(-0.5);
        } else if (dPadRight) {
//            Strafe Right
            Flm.setPower(-0.5);
            Brm.setPower(-0.5);
            Frm.setPower(0.5);
            Blm.setPower(0.5);

        } else if (dPadUp) {
            //            Drive Forward
            Flm.setPower(0.5);
            Brm.setPower(-0.5);
            Frm.setPower(0.5);
            Blm.setPower(-0.5);
        } else if (dPadDown) {
            //            Drive Backward
            Flm.setPower(-0.5);
            Brm.setPower(0.5);
            Frm.setPower(-0.5);
            Blm.setPower(0.5);
        }

        telemetry.addData("Status", "Running");

        telemetry.update();
    }

    public void stop() {
        Frm.setPower(0);
        Blm.setPower(0);
        Brm.setPower(0);
        Flm.setPower(0);
        clawRotater.setPosition(0);
        claw.setPosition(0);
    }
}
