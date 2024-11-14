package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.CRServo;

// OpenCV Imports
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;

@TeleOp(name="RevHub Controller with Camera", group="TeleOp")
public class Main extends OpMode {
    // Motor declarations
    private DcMotorEx Frm, Blm, Brm, Flm;
    private DcMotorEx RobotLiftLeft, RobotLiftRight;
//    private DcMotorEx leftEncoder, rightEncoder, backEncoder;
    private DcMotorEx ArmLift;

//    Servo Declarations
    private Servo armActuator;

// The intake uses a servo programmed to be continuous to spin an intake wheel
    private CRServo armActuatorIntake;

//    Target Powers
    private double tgtLeftJoy = 0;
    private double tgtRightJoy = 0;
    private double tgtArmLift = 0;
    private double tgtArmActuator = 0;

    // Localizer
//    private CustomThreeWheelLocalizer localizer;

    private FtcDashboard dashboard;

    // OpenCV Camera
    private OpenCvWebcam webcam;

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
    boolean a;
    boolean b;
    boolean x;
    boolean y;

    // Controller Variables
    double leftStickDeadZone = 0.05;

//    Arm Actuator Variables, the arm actuator is a servo that controls a servo lift based on the angle of the servo, the closer the servo variable gets to its maximum the more extended the actuator gets, the closer it gets to the minimum, the more the scissor lift contracts, d-pad up makes the intake spin in and d-pad down makes the intake spin out
    private final double actMin = 0;
    private final double actMax = 55;
    private double actRange = actMax - actMin;

    @Override
    public void init() {
        // Motor initialization
        Frm = hardwareMap.get(DcMotorEx.class, "Frm");
        Blm = hardwareMap.get(DcMotorEx.class, "Blm");
        Brm = hardwareMap.get(DcMotorEx.class, "Brm");
        Flm = hardwareMap.get(DcMotorEx.class, "Flm");
        ArmLift = hardwareMap.get(DcMotorEx.class, "ArmLift");
        RobotLiftLeft = hardwareMap.get(DcMotorEx.class, "RobotLiftLeft");
        RobotLiftRight = hardwareMap.get(DcMotorEx.class, "RobotLiftRight");
        armActuatorIntake = hardwareMap.get(CRServo.class, "ArmActuatorIntake");

        // Servo initialization
        armActuator = hardwareMap.get(Servo.class, "ArmActuator");

        // Encoder initialization
//        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
//        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
//        backEncoder = hardwareMap.get(DcMotorEx.class, "backEncoder");

        // Motor directions and modes
        Frm.setDirection(DcMotorEx.Direction.REVERSE);
        Blm.setDirection(DcMotorEx.Direction.FORWARD);
        Brm.setDirection(DcMotorEx.Direction.REVERSE);
        Flm.setDirection(DcMotorEx.Direction.REVERSE);

        Frm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Blm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Brm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Flm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        RobotLiftLeft.setDirection(DcMotorEx.Direction.FORWARD);
        RobotLiftRight.setDirection(DcMotorEx.Direction.FORWARD);
        RobotLiftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RobotLiftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ArmLift.setDirection(DcMotorEx.Direction.FORWARD);
        ArmLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armActuatorIntake.setDirection(DcMotorEx.Direction.FORWARD);

        armActuator.setDirection(Servo.Direction.FORWARD);
        armActuator.setPosition(0);

//        localizer = new CustomThreeWheelLocalizer(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Set the pipeline
        webcam.setPipeline(new SamplePipeline());

        // Open the camera device asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming images from the camera
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                // Stream the camera feed to the dashboard
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened");
            }
        });

        String ipAddress = getRobotControllerIpAddress();
        String dashboardUrl = "http://" + ipAddress + ":8080/dash"; // FTC Dashboard runs on port 8080

        // Display the URL and dashboard status on telemetry
        telemetry.addData("FTC Dashboard URL", dashboardUrl);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public String getRobotControllerIpAddress() {
        try {
            List<NetworkInterface> interfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
            for (NetworkInterface networkInterface : interfaces) {
                if (networkInterface.isUp() && !networkInterface.isLoopback()) {
                    List<InetAddress> addresses = Collections.list(networkInterface.getInetAddresses());
                    for (InetAddress address : addresses) {
                        if (!address.isLoopbackAddress() && !address.isLinkLocalAddress() && address instanceof InetAddress) {
                            String ip = address.getHostAddress();
                            if (ip != null && !ip.contains(":")) { // Exclude IPv6 addresses
                                return ip;
                            }
                        }
                    }
                }
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Could not get IP address");
        }
        return "Unknown";
    }

    @Override
    public void loop() {
        // Controller input processing
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

        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        y = gamepad1.y;


        // Speed and Control Variables
        double frmSpeed = 0;
        double blmSpeed = 0;
        double brmSpeed = 0;
        double flmSpeed = 0;

//        Make motors move forward based on A, B, X, Y
        if (a) {
            tgtArmLift = 1;
        } else if (b) {
            tgtArmLift = -1;
        } else {
            tgtArmLift = 0;
        }

//        Tank Drive Controls
        if (Math.abs(leftStickY) > leftStickDeadZone) {
            flmSpeed = rightStickY;
            blmSpeed = rightStickY;
        }
        if (Math.abs(rightStickY) > leftStickDeadZone) {
            frmSpeed = leftStickY;
            brmSpeed = leftStickY;
        }

//        Arm Actuator Controls
        //        Use the right trigger to control the arm actuator, the servo extension amount depends on how far the trigger is pressed.
        tgtArmActuator = rightTrigger;
//        Use trig and kinematics to ensure that arm actuator extends at constant speed, since the servo is attached to one bar which is attached to another which is then attached to the arm actuator, it will move at different speeds depending on the position, this needs to be corrected
        armActuator.setPosition(calculateArmActuatorServoPos(tgtArmActuator));

//      Control Intake Speed with Left Trigger and intake direction with d-pad
        double targetIntakeSpeed = 0;

        if (leftTrigger > 0) {
            if (dPadDown) {
                targetIntakeSpeed = leftTrigger;
            } else {
                targetIntakeSpeed = -leftTrigger;
            }
        }

//        Arm Lift Controls
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

//        Set Power
        Frm.setPower(frmSpeed);
        Blm.setPower(blmSpeed);
        Brm.setPower(brmSpeed);
        Flm.setPower(flmSpeed);
        ArmLift.setPower(tgtArmLift);
        armActuatorIntake.setPower(targetIntakeSpeed);

        // Update the localizer
//        localizer.update();

        // Get the current estimated pose
//        Pose2d poseEstimate = localizer.getPoseEstimate();

        // Setup packet to send telemetry data to the dashboard
        TelemetryPacket packet = new TelemetryPacket();

        // Add pose data to telemetry
//        telemetry.addData("Pose X", poseEstimate.getX());
//        telemetry.addData("Pose Y", poseEstimate.getY());
//        telemetry.addData("Pose Heading", Math.toDegrees(poseEstimate.getHeading()));

        // Add pose data to dashboard packet
//        packet.put("Pose X", poseEstimate.getX());
//        packet.put("Pose Y", poseEstimate.getY());
//        packet.put("Pose Heading", Math.toDegrees(poseEstimate.getHeading()));

        // Send telemetry
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when the op mode ends
        Frm.setPower(0);
        Blm.setPower(0);
        Brm.setPower(0);
        Flm.setPower(0);
        armActuator.setPosition(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();

        // Stop the camera
        if (webcam != null) {
            webcam.stopStreaming();
            FtcDashboard.getInstance().stopCameraStream();
        }
    }
    public double calculateArmActuatorServoPos(double targetPosition) {
        return actMin + (targetPosition / actRange) * (actMax - actMin);
    }

    public double getActRange() {
        return actRange;
    }

    public void setActRange(double actRange) {
        this.actRange = actRange;
    }

    // Sample OpenCV Pipeline
    public class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Process the input frame (e.g., image processing tasks)

            // For now, just return the input frame
            return input;
        }
    }
}

