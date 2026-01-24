package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Parameters {
    public static Pose2d startPose = new Pose2d(-60, -52, Math.toRadians(225));

    public static double drumP = 0.004;
    public static double drumI = 0.0;
    public static double drumD = 0.0000;
    public static double drumF = 0.0000;
    public static double drumPower = 0.5;  // Max power for drum movement (0.0 to 1.0); tune for speed vs. precision
    public static double closeRPM = 1800;
    public static double farRPM = 2125;
    public static double driveGain = 2000;
    public static double driveRotGain = .6;

    public static  int IN_TO_OUT_OFFSET = 2010; //1750;//4096;
    public static  int POCKET1_IN = 0;
    public static  int POCKET2_IN = 1350;//268; //1167;//2730;
    public static  int POCKET3_IN = 2650;//518; //2333;//5461;

    public static int pocketTarget = 1;
    public static int drum_in_out = 1;
    public static int correction = 0;
    public static int intakeManual = 0;

}
