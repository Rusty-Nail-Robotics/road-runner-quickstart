package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config

public class RedNearParameters {
    public static volatile boolean launching = false;
    public static boolean autoIndexEnabled = false;
    //START POSITION
    public static Pose2d startPose = new Pose2d(-60, 52, Math.toRadians(135));

    //FIRST LAUNCH LOCATION
    public static Vector2d launchLocation = new Vector2d(-15,15);
    public static double launchHeading = 130;
    public static double launchRPM = 1750;

    //FIRST GRAB LOCATION
    public static Pose2d firstGrab = new Pose2d(-18, 30, Math.toRadians(90));
    public static double intakeForwardDistance = 15;
    public static double intakeSpeed = 5;
    //public static double x2 = 60;
    //public static double y2 = 0;
    //public static double h2 = 25;
}
