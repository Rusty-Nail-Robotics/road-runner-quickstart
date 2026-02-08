package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeControl {

    // Tunable constants
    private static final double DETECTION_THRESHOLD_INCHES = 9.0; // Distance to start intake (tune as needed)
    private static final long RUN_AFTER_LOSS_MS = 3000; // 8 seconds run after detection lost (tune)
    private static final long REVERSE_DURATION_MS = 3000; // 3 seconds reverse to spit out (tune)
    private static final double INTAKE_POWER = -1.0; // Forward power (0-1.0; tune for speed)
    private static final double REVERSE_POWER = 1.0; // Reverse power (negative; tune)

    private DcMotorEx intakeMotor;
    private DistanceSensor intakeSensor;

    // State for non-blocking timers
    private long lastDetectionTime = 0;
    private boolean intakeRunning = false;
    private boolean reversing = false;
    private long reverseStartTime = 0;

    // Constructor: Initialize with HardwareMap
    public IntakeControl(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor"); // REV 2m Distance Sensor

        // Configure motor
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Simple power control
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake when off
    }

    // Update method: Call in main OpMode loop (non-blocking)
    public void update(Sensors sensorDisplay) {
        double distanceInches = intakeSensor.getDistance(DistanceUnit.INCH);
        if ((distanceInches < DETECTION_THRESHOLD_INCHES && Parameters.drum_in_out == 1 ) || Parameters.intakeManual == 1) {
            // Object detected: Start/keep running and reset timer
            intakeRunning = true;
            lastDetectionTime = System.currentTimeMillis();
        } else if (intakeRunning) {
            // No detection: Check if timer expired
            if (System.currentTimeMillis() - lastDetectionTime >= RUN_AFTER_LOSS_MS) {
                intakeRunning = false;
            }
        }

        // Handle reversing (overrides intake)
        if (reversing) {
            if (System.currentTimeMillis() - reverseStartTime >= REVERSE_DURATION_MS) {
                reversing = false;
                intakeMotor.setPower(0.0); // Stop after reverse
            } else {
                intakeMotor.setPower(REVERSE_POWER);
            }
        } else if (intakeRunning) {
            intakeMotor.setPower(INTAKE_POWER); // Forward run
        } else {
            intakeMotor.setPower(0.0); // Off
        }
    }
    public void StopIntake(){
        intakeRunning = false;
        reversing = false;
    }

    // Function to start reverse (non-blocking; update() handles timing)
    public void startReverse() {
        if (!reversing) {
            reversing = true;
            reverseStartTime = System.currentTimeMillis();
            intakeRunning = false; // Override normal intake
        }
    }

    // Optional: Check if intake is active (for telemetry/logic)
    public boolean isIntakeRunning() {
        return intakeRunning || reversing;
    }

    // Optional: Get current distance for telemetry
    public double getIntakeDistanceInches() {
        return intakeSensor.getDistance(DistanceUnit.INCH);
    }
}