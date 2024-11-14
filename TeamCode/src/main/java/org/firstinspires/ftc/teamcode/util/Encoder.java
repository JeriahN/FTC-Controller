// File: Encoder.java
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    public enum Direction {
        FORWARD,
        REVERSE
    }

    private DcMotorEx motor;
    private Direction direction = Direction.FORWARD;

    public Encoder(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int position = motor.getCurrentPosition();
        return direction == Direction.FORWARD ? position : -position;
    }

    public double getCorrectedVelocity() {
        double velocity = motor.getVelocity();
        return direction == Direction.FORWARD ? velocity : -velocity;
    }
}
