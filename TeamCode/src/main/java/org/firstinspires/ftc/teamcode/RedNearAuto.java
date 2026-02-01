package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Basket Auto", group = "Autonomous")
public class RedNearAuto extends LinearOpMode {

    private DrumIndexer indexer;
    private LauncherControl launcherControl;
    private MecanumDrive drive;
    private IntakeControl intakeControl;
    private SensorDisplay sensorDisplay;

    @Override
    public void runOpMode() {
        // Starting pose


        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, RedNearParameters.startPose);
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

        Action main = drive.actionBuilder(RedNearParameters.startPose)
                .strafeTo(RedNearParameters.launchLocation)
                //.turnTo(Math.toRadians(RedNearParameters.launchHeading))
                //.stopAndAdd(new LaunchCycleAction(indexer,launcherControl, drive, RedNearParameters.launchRPM))
                //.turnTo(Math.toRadians(90))//270 Flipped from blue
                //.splineToSplineHeading(RedNearParameters.firstGrab,Math.toRadians(100))//280
                //.stopAndAdd(new SetAutoIndexEnabledAction(true))
                // New: Slow drive forward for intake (adjust distance/speed)
                //.lineToY(RedNearParameters.firstGrab.position.y + RedNearParameters.intakeForwardDistance,new TranslationalVelConstraint(RedNearParameters.intakeSpeed),new ProfileAccelConstraint(-5,5)
                //)
                //.stopAndAdd(new SetAutoIndexEnabledAction(false))
                // New: Return to launch and fire again
                //.strafeTo(RedNearParameters.launchLocation)
                //.turnTo(Math.toRadians(RedNearParameters.launchHeading))
                //.stopAndAdd(new LaunchCycleAction(indexer, launcherControl, drive, RedNearParameters.launchRPM))  // Second launch cycle
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