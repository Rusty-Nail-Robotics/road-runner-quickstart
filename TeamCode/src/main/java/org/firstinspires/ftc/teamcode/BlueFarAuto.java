package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Far Auto", group = "Autonomous")
public class BlueFarAuto extends LinearOpMode {

    private DrumIndexer indexer;
    private LauncherControl launcherControl;
    private MecanumDrive drive;
    private IntakeControl intakeControl;
    private SensorDisplay sensorDisplay;

    @Override
    public void runOpMode() {
        // Starting pose


        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, BlueFarParameters.startPose);
        indexer = new DrumIndexer(hardwareMap);
        launcherControl = new LauncherControl(hardwareMap);
        intakeControl = new IntakeControl(hardwareMap);
        sensorDisplay = new SensorDisplay(hardwareMap);

        telemetry.addLine("Basic Autonomous Ready");
        telemetry.update();

        waitForStart();

        // Build and run trajectory to target pose

        Action main = drive.actionBuilder(BlueFarParameters.startPose)
                .strafeTo(BlueFarParameters.launchLocation)
                .turnTo(Math.toRadians(BlueFarParameters.launchHeading))
                .stopAndAdd(new LaunchCycleAction(indexer,launcherControl, drive, BlueFarParameters.launchRPM))
                .splineToSplineHeading(BlueFarParameters.firstGrab,Math.toRadians(270))
                .stopAndAdd(new SetAutoIndexEnabledAction(true))
                // New: Slow drive forward for intake (adjust distance/speed)
                                .lineToY(BlueFarParameters.firstGrab.position.y + BlueFarParameters.intakeForwardDistance,new TranslationalVelConstraint(BlueFarParameters.intakeSpeed),new ProfileAccelConstraint(-5,5)
                )
                .stopAndAdd(new SetAutoIndexEnabledAction(false))
                // New: Return to launch and fire again
                .strafeTo(BlueFarParameters.launchLocation)
                .turnTo(Math.toRadians(BlueFarParameters.launchHeading))
                .stopAndAdd(new LaunchCycleAction(indexer, launcherControl, drive, BlueFarParameters.launchRPM))  // Second launch cycle
                .build();

        Actions.runBlocking(
                new RaceAction(
                        main,
                        new SubsystemUpdateAction(indexer, intakeControl, sensorDisplay, launcherControl, drive)

                )

        );


        // Keep updating until opmode stops
        while (opModeIsActive()) {
            indexer.update(sensorDisplay);
            indexer.updatePush();
            drive.updatePoseEstimate();
            intakeControl.update();
            telemetry.update();
        }}}