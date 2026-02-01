package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Basket Auto", group = "Autonomous")
public class BlueNearAuto extends LinearOpMode {

    private DrumIndexer indexer;
    private LauncherControl launcherControl;
    private MecanumDrive drive;
    private IntakeControl intakeControl;
    private SensorDisplay sensorDisplay;

    @Override
    public void runOpMode() {
        // Starting pose


        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, BlueNearParameters.startPose);
        indexer = new DrumIndexer(hardwareMap);
        launcherControl = new LauncherControl(hardwareMap);
        intakeControl = new IntakeControl(hardwareMap);
        sensorDisplay = new SensorDisplay(hardwareMap);

        telemetry.addLine("Basic Autonomous Ready");
        telemetry.update();
        indexer.inBlock.setPosition(1.0);
        Parameters.drum_in_out = 0;
        waitForStart();

        // Build and run trajectory to target pose

        Action main = drive.actionBuilder(BlueNearParameters.startPose)
                .strafeTo(BlueNearParameters.launchLocation)
                //.turnTo(Math.toRadians(BlueNearParameters.launchHeading))
                //.stopAndAdd(new LaunchCycleAction(indexer,launcherControl, drive, BlueNearParameters.launchRPM))
                //.turnTo(Math.toRadians(270))
                //.splineToSplineHeading(BlueNearParameters.firstGrab,Math.toRadians(280))
                //.stopAndAdd(new SetAutoIndexEnabledAction(true))
                // New: Slow drive forward for intake (adjust distance/speed)
                //.lineToY(BlueNearParameters.firstGrab.position.y + BlueNearParameters.intakeForwardDistance,new TranslationalVelConstraint(BlueNearParameters.intakeSpeed),new ProfileAccelConstraint(-5,5)
                //)
                //.stopAndAdd(new SetAutoIndexEnabledAction(false))
                // New: Return to launch and fire again
                //.strafeTo(BlueNearParameters.launchLocation)
                //.turnTo(Math.toRadians(BlueNearParameters.launchHeading))
               // .stopAndAdd(new LaunchCycleAction(indexer, launcherControl, drive, BlueNearParameters.launchRPM))  // Second launch cycle
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
            Parameters.startPose = drive.localizer.getPose();
            intakeControl.update(sensorDisplay);
            telemetry.update();
        }}}