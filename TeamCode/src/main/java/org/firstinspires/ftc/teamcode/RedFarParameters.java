package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config

public class RedFarParameters {
    public static volatile boolean launching = false;
    public static boolean autoIndexEnabled = false;
    //START POSITION
    public static Pose2d startPose = new Pose2d(68, 24, Math.toRadians(180));

    //FIRST LAUNCH LOCATION
    public static Vector2d launchLocation = new Vector2d(60,20);
    public static double launchHeading = 160;
    public static double launchRPM = 1800;

    //FIRST GRAB LOCATION
    public static Pose2d firstGrab = new Pose2d(36, 25, Math.toRadians(90));
    public static double intakeForwardDistance = 20;
    public static double intakeSpeed = 5;
    //public static double x2 = 60;
    //public static double y2 = 0;
    //public static double h2 = 25;
}
