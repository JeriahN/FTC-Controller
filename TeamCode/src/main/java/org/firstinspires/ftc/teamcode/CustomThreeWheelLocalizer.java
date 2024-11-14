// File: CustomThreeWheelLocalizer.java
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class CustomThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // Constants (adjust these to match your hardware)
    public static double TICKS_PER_REV = 8192; // For REV Through Bore Encoders
    public static double WHEEL_RADIUS = 0.6889764; // in inches (35mm diameter)
    public static double GEAR_RATIO = 1; // Ratio between encoder and wheel (usually 1:1)

    // Wheel positions (adjust based on your robot's dimensions)
    public static double LATERAL_DISTANCE = 13.0; // Distance between the left and right wheels
    public static double FORWARD_OFFSET = -8.5;   // Offset of the back wheel along the Y-axis

    // Encoder hardware
    private Encoder leftEncoder, rightEncoder, backEncoder;

    public CustomThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                // Left wheel pose (X, Y, Heading)
                new Pose2d(-6.5, 3.9, Math.toRadians(90)), // Left wheel
                // Right wheel pose
                new Pose2d(6.5, 3.9, Math.toRadians(90)),  // Right wheel
                // Back wheel pose
                new Pose2d(1.5, -8.5, Math.toRadians(0))    // Back wheel
        ));

        // Initialize encoders
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));

        // Reverse encoders if necessary (adjust based on testing)
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        backEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    // Convert encoder ticks to inches
    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    // Return the positions of the wheels (in inches)
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(backEncoder.getCurrentPosition())
        );
    }

    // Return the velocities of the wheels (in inches per second)
    @Override
    public List<Double> getWheelVelocities() {
        // Use getCorrectedVelocity() if available
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(backEncoder.getCorrectedVelocity())
        );
    }
}
