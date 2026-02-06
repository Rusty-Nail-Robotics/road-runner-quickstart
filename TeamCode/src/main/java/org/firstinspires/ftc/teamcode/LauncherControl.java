package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherControl {

    // Constants
    private static final double TICKS_PER_REV = 28.0; // Adjust based on your launcher motors (e.g., 28 for REV HD Hex, 537.6 for GoBILDA 5202)

    private DcMotorEx launcherLeft;
    private DcMotorEx launcherRight;

    // Constructor: Initialize with HardwareMap
    public LauncherControl(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        // Configure motors for velocity control
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Tune built-in PIDF if needed (defaults are often fine for launchers)
        // launcherLeft.setVelocityPIDFCoefficients(5.0, 0.1, 0.0, 10.0);
        // launcherRight.setVelocityPIDFCoefficients(5.0, 0.1, 0.0, 10.0);

        // Assume forward direction; reverse if needed
         launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Update(MecanumDrive drive){
        if(Parameters.launcherOn){
            if (Parameters.launcherHigh){setRPM(Parameters.farRPM);}
            else{setRPM(Parameters.closeRPM);}
            //double currentLocationX = drive.localizer.getPose().position.x;
            // if(currentLocationX < -30){launcherControl.setRPM(Parameters.farRPM);}
            // if(currentLocationX > -30){launcherControl.setRPM(Parameters.closeRPM);}
        }else{
            setRPM(0);
        }
    }


    // Set RPM for both motors (0 to turn off)
    public void setRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV; // Convert RPM to ticks/sec
        launcherLeft.setVelocity(ticksPerSec);
        launcherRight.setVelocity(ticksPerSec);
    }

    // Optional: Get current average RPM (for telemetry/debug)
    public double getCurrentRPM() {
        double leftTicksSec = launcherLeft.getVelocity();
        double rightTicksSec = launcherRight.getVelocity();
        double avgTicksSec = (leftTicksSec + rightTicksSec) / 2.0;
        return (avgTicksSec / TICKS_PER_REV) * 60.0; // Convert to RPM
    }
}