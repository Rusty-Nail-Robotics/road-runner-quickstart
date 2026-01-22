package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.KeyStore;

public class DrumIndexer {

    // Enums for pocket and port selection
    public enum Pocket {
        ONE,
        TWO,
        THREE
    }

    public enum Port {
        IN,
        OUT
    }

    // Constants from your code



    private DcMotorEx drum;
    private Servo pusher; // Optional, if needed for pushing
    private Servo outBlock;
    private Servo inBlock;
    private PIDFController drumPIDF;
    private int targetPosition = 0;

    // Non-blocking push state
    private boolean pushing = false;
    private long pushStartTime = 0;
    private static final long EXTEND_HOLD_MS = 300; // Time to hold extended (tune for push)
    private static final long RETRACT_DELAY_MS = 500; // Delay before retracting (added per request)

    private boolean retracting = false;
    private long retractStartTime = 0;
    public int pocketTarget = 1;
    public int drum_in_out = 1;

    // Constructor: Initialize with HardwareMap
    public DrumIndexer(HardwareMap hardwareMap) {
        drum = hardwareMap.get(DcMotorEx.class, "indexDrum");
        pusher = hardwareMap.get(Servo.class, "pusher");
        outBlock = hardwareMap.get(Servo.class, "outBlock");
        inBlock = hardwareMap.get(Servo.class, "inBlock");

        drum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // needed for velocity readout

        // Initialize PIDF with your values
        drumPIDF = new PIDFController(Parameters.drumP, Parameters.drumI, Parameters.drumD, Parameters.drumF);


        pusher.setPosition(0.0); // Retracted position
        outBlock.setPosition(1);
        inBlock.setPosition(0.0); // Assume 0.0 is open (allow intake); tune if reversed
    }

    // Method to select pocket and port alignment
    public void setAlignment(Pocket pocket, Port port) {
        int basePosition = 0;
        switch (pocket) {
            case ONE:
                basePosition = Parameters.POCKET1_IN;
                pocketTarget = 1;
                break;
            case TWO:
                basePosition = Parameters.POCKET2_IN;
                pocketTarget = 2;
                break;
            case THREE:
                basePosition = Parameters.POCKET3_IN;
                pocketTarget = 3;
                break;
        }

        if (port == Port.OUT) {
            basePosition += Parameters.IN_TO_OUT_OFFSET;
            drum_in_out = 0;
        }
        else{drum_in_out = 1;}

        targetPosition = basePosition;
    }

    // Update method: Call this in the main loop to apply power
    public void update(SensorDisplay sensorDisplay) {
        drumPIDF.setPIDF(Parameters.drumP, Parameters.drumI, Parameters.drumD, Parameters.drumF);
        double currentPosition = drum.getCurrentPosition();
        double outputPower = drumPIDF.calculate(currentPosition, targetPosition);
        drum.setPower(outputPower);

        // Block intake if drum is rotating (not stable)
        if (sensorDisplay.GetDetectedDistance() < 100 && drum_in_out == 1) {
            inBlock.setPosition(1.0); // Assume 1.0 is block (close); tune if reversed
        } else {
            inBlock.setPosition(0.2); // Open (allow intake)
        }
    }

    public boolean DrumAtTarget(){
        if (Math.abs(drum.getVelocity()) <= 500 && Math.abs(drum.getCurrentPosition() - targetPosition) <= 300){
            return true;
        }else{
            return false;
        }
    }

    // Wider tolerance for servo to reduce jitter during settling
    public boolean DrumStableForServo(){
        if (Math.abs(drum.getCurrentPosition() - targetPosition) <= 70){ // Twice as wide as DrumAtTarget
            return true;
        }else{
            return false;
        }
    }

    // Optional: Get current target for telemetry
    public int getTargetPosition() {
        return targetPosition;
    }

    // Optional: Get current drum position for telemetry
    public int getCurrentPosition() {
        return drum.getCurrentPosition();
    }

    // Non-blocking: Start the push action
    public void startPush() {
        if (!pushing && !retracting) {
            outBlock.setPosition(.5);
            pusher.setPosition(1.0); // Extend to push
            pushStartTime = System.currentTimeMillis();
            pushing = true;
        }
    }

    // Non-blocking update for push (call in main loop after update())
    public void updatePush() {
        if (pushing) {
            if (System.currentTimeMillis() - pushStartTime >= EXTEND_HOLD_MS + RETRACT_DELAY_MS) {
                outBlock.setPosition(1);
                pusher.setPosition(0.0); // Retract after delay
                retractStartTime = System.currentTimeMillis();
                pushing = false;
                retracting = true;
            }
        } else if (retracting) {
            if (System.currentTimeMillis() - retractStartTime >= 1000) { // Brief retract settle
                retracting = false;
            }
        }
    }

    // Check if push/retract cycle is complete (for sequencing)
    public boolean isPushComplete() {
        return !pushing && !retracting;
    }

}