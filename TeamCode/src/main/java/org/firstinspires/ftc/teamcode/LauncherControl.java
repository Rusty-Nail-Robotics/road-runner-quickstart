package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class LauncherControl {

    // Constants
    private static final double TICKS_PER_REV = 28.0*3; // Adjust based on your launcher motors (e.g., 28 for REV HD Hex, 537.6 for GoBILDA 5202)

    public DcMotorEx launcherLeft;

    public DcMotorEx launcherRight;
    public DigitalChannel highLowIndicator1R;
    public DigitalChannel highLowIndicator1G;
    public DigitalChannel highLowIndicator2R;
    public DigitalChannel highLowIndicator2G;
    public DigitalChannel highLowIndicator3R;
    public DigitalChannel highLowIndicator3G;
    public DigitalChannel highLowIndicator4R;
    public DigitalChannel highLowIndicator4G;

    public static double ticksPerSec = 0;

    // Constructor: Initialize with HardwareMap
    public LauncherControl(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        highLowIndicator1R = hardwareMap.get(DigitalChannel.class, "lowHigh1R");
        highLowIndicator1G = hardwareMap.get(DigitalChannel.class, "lowHigh1G");

        highLowIndicator2R = hardwareMap.get(DigitalChannel.class, "lowHigh2R");
        highLowIndicator2G = hardwareMap.get(DigitalChannel.class, "lowHigh2G");

        highLowIndicator3R = hardwareMap.get(DigitalChannel.class, "lowHigh3R");
        highLowIndicator3G = hardwareMap.get(DigitalChannel.class, "lowHigh3G");

        highLowIndicator4R = hardwareMap.get(DigitalChannel.class, "lowHigh4R");
        highLowIndicator4G = hardwareMap.get(DigitalChannel.class, "lowHigh4G");
        highLowIndicator1R.setMode(DigitalChannel.Mode.OUTPUT);
        highLowIndicator1G.setMode(DigitalChannel.Mode.OUTPUT);

        highLowIndicator2R.setMode(DigitalChannel.Mode.OUTPUT);
        highLowIndicator2G.setMode(DigitalChannel.Mode.OUTPUT);

        highLowIndicator3R.setMode(DigitalChannel.Mode.OUTPUT);
        highLowIndicator3G.setMode(DigitalChannel.Mode.OUTPUT);

        highLowIndicator4R.setMode(DigitalChannel.Mode.OUTPUT);
        highLowIndicator4G.setMode(DigitalChannel.Mode.OUTPUT);

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

    public void SetLauncherLEDHigh(boolean state){
        if(state == true){
            highLowIndicator1R.setState(false);
            highLowIndicator1G.setState(true);
            highLowIndicator2R.setState(false);
            highLowIndicator2G.setState(true);
            highLowIndicator3R.setState(false);
            highLowIndicator3G.setState(true);
            highLowIndicator4R.setState(false);
            highLowIndicator4G.setState(true);
        }else{
            highLowIndicator1R.setState(true);
            highLowIndicator1G.setState(false);
            highLowIndicator2R.setState(true);
            highLowIndicator2G.setState(false);
            highLowIndicator3R.setState(true);
            highLowIndicator3G.setState(false);
            highLowIndicator4R.setState(true);
            highLowIndicator4G.setState(false);
        }
    }

    public void Update(MecanumDrive drive){


            if (Parameters.launcherHigh){
                if(Parameters.launcherOn) {
                    setRPM(Parameters.farRPM);
                }
                SetLauncherLEDHigh(true);
            }
            else{
                if(Parameters.launcherOn) {
                    setRPM(Parameters.closeRPM);
                }
                SetLauncherLEDHigh(false);


        }

            if(!Parameters.launcherOn){
                setRPM(0);
            }
       // launcherLeft.setPower(leftPID.PIDControl(500, launcherLeft.getVelocity()));
        //launcherRight.setPower(rightPID.PIDControl(500, launcherRight.getVelocity()));
    }


    // Set RPM for both motors (0 to turn off)
    public void setRPM(double rpm) {
        ticksPerSec = (rpm / 60.0) * TICKS_PER_REV; // Convert RPM to ticks/sec

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