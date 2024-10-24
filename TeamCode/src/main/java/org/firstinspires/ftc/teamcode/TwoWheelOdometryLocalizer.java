package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class TwoWheelOdometryLocalizer extends TwoTrackingWheelLocalizer {
//    Standard https://www.revrobotics.com/rev-11-1271/

    // Physical constants
    public static double TICKS_PER_REV = /* Your encoder's ticks per revolution */;
    public static double WHEEL_RADIUS = /* Wheel radius in inches */;
    public static double GEAR_RATIO = 1; // If there's no gearing between the encoder and wheel

    // Positions of the tracking wheels relative to the robot's center
    public static double PARALLEL_X = /* X coordinate of the parallel wheel */;
    public static double PARALLEL_Y = /* Y coordinate of the parallel wheel */;
    public static double PERPENDICULAR_X = /* X coordinate of the perpendicular wheel */;
    public static double PERPENDICULAR_Y = /* Y coordinate of the perpendicular wheel */;

    // The encoders
    private DcMotorEx parallelEncoder, perpendicularEncoder;

    public TwoWheelOdometryLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0), // Position of the parallel wheel
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90)) // Position of the perpendicular wheel
        ));

        // Initialize encoders from hardware map
        parallelEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");

        // Reverse encoders if necessary
        parallelEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return 0;
    }
}
