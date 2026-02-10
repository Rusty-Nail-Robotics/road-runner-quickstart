package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Parameters {
    public static int drumtesttarget = 0;

    public static Pose2d startPose = new Pose2d(-60, -52, Math.toRadians(225));
    public static boolean telemetryOutput = false; //TODO: Set to False for Competition
    public static boolean coldStart = true; //TODO: Set to False for Competition
    public static double drumP = 0.0045;//0.01;
    public static double drumI = 0;//0.00001;
    public static double drumD = 0.000001;//0.0002;
    public static double drumF = 0.0000;
    public static int drumLastPosition = 0;

    private static double gearRatio = 3.61;
    public static int pocketLocation0 =  (int)Math.round(0 * gearRatio);//intake pocket 1
    public static int pocketLocation1 = (int)Math.round(131 * gearRatio);//Output pocket 2
    public static int pocketLocation2 = (int)Math.round(261 * gearRatio);//intake pocket 3
    public static int pocketLocation3 = (int)Math.round(392 * gearRatio);//output Pocket 1
    public static int pocketLocation4 = (int)Math.round(522 * gearRatio);//intake pocket 2
    public static int pocketLocation5 = (int)Math.round(653 * gearRatio);//output pocket 3

   // public static double drumPower = 0.5;  // Max power for drum movement (0.0 to 1.0); tune for speed vs. precision
    public static double closeRPM = 1650;
    public static double farRPM = 2050;
    public static double driveGain = 2000;
    public static double driveRotGain = .6;


    public static int pocketTarget = 1;
    public static int drum_in_out = 1;
    public static int correction = 0;
    public static int intakeManual = 0;

    public static boolean launcherOn = false;
    public static boolean launcherHigh = false;
    public static double launchDelayMS = 500;
    public static int telemetryIntervalMs = 500;
    public static boolean autoIndexEnabled = false;


    /// //////// launcher Pids



}
