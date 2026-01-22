package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriverMecanum {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    // Constructor: Initialize motors with HardwareMap
    public DriverMecanum(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Configure motor directions (adjust if needed for your robot's wiring/orientation)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set modes (simple power for drive; change to RUN_USING_ENCODER if using velocity)
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Update drive powers from gamepad (call in main loop)
    public void update(Gamepad gamepad) {
        // Controls per user: Left analog x for rotate (turn rx), Right analog y for forward (y, negate for inverted), x for strafe
        double rx = gamepad.left_stick_x*Parameters.driveRotGain; // Left analog: rotate left/right
        double y = -gamepad.right_stick_y; // Right analog: forward/back (negate if inverted)
        double x = gamepad.right_stick_x; // Right analog: strafe left/right

        // Mecanum calculations
        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;

        // Normalize if any >1 (prevent clipping)
        double max = Math.max(1.0, Math.abs(leftFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Set powers
        leftFront.setVelocity(leftFrontPower*Parameters.driveGain);
        leftBack.setVelocity(leftBackPower*Parameters.driveGain);
        rightFront.setVelocity(rightFrontPower*Parameters.driveGain);
        rightBack.setVelocity(rightBackPower*Parameters.driveGain);
    }

    // Optional: Stop all motors
    public void stop() {
        leftFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightFront.setVelocity(0);
        rightBack.setVelocity(0);
    }
}