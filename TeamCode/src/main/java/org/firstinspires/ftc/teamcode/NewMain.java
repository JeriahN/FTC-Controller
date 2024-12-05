package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="NewMain", group="Linear Opmode")
public class NewMain extends LinearOpMode {

    private DcMotorEx frm, blm, brm, flm;
    private Servo clawRotater, claw;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        frm = hardwareMap.get(DcMotorEx.class, "Frm");
        blm = hardwareMap.get(DcMotorEx.class, "Blm");
        brm = hardwareMap.get(DcMotorEx.class, "Brm");
        flm = hardwareMap.get(DcMotorEx.class, "Flm");

        clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.FORWARD);
        flm.setDirection(DcMotorSimple.Direction.FORWARD);

        double joystickleftX = gamepad1.left_stick_x;
        double joysticklefty = gamepad1.left_stick_y;
        double joystickrightx = gamepad1.right_stick_x;
        double joystickrighty = gamepad1.right_stick_y;

        boolean dPadUp = gamepad1.dpad_up;
        boolean dPadDown = gamepad1.dpad_down;
        boolean dPadLeft = gamepad1.dpad_left;
        boolean dPadRight = gamepad1.dpad_right;

        boolean bButton = gamepad1.b;
        boolean xButton = gamepad1.x;
        boolean aButton = gamepad1.a;
        boolean yButton = gamepad1.y;

        double frmPower = joystickrighty;
        double blmPower = joystickrighty;
        double brmPower = joysticklefty;
        double flmPower = joysticklefty;

        waitForStart();

        while (opModeIsActive()) {
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
            frm.setPower(frmPower);
            blm.setPower(blmPower);
            brm.setPower(brmPower);
            flm.setPower(flmPower);

            // Control claw rotater with button B
            if (bButton) {
                clawRotater.setPosition(1);
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
                claw.setPosition(claw.getPosition() + 0.1);
                telemetry.addData("Claw Position", claw.getPosition());
            } else if (yButton) {
                claw.setPosition(claw.getPosition() - 0.1);
                telemetry.addData("Claw Position", claw.getPosition());
            } else {
                claw.setPosition(claw.getPosition());
                telemetry.addData("Claw Position", claw.getPosition());
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}