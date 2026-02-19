package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Wall Auto", group = "Autonomous")
public class RedFarAuto extends LinearOpMode {

    private DrumIndexer indexer;
    private LauncherControl launcherControl;
    private MecanumDrive drive;
    private IntakeControl intakeControl;
    private Sensors sensorDisplay;

    @Override
    public void runOpMode() {
        // Starting pose


        // Initialize hardware
        Parameters.coldStart = true;
        drive = new MecanumDrive(hardwareMap, RedFarParameters.startPose);
        indexer = new DrumIndexer();
        indexer.DrumIndexerInit(hardwareMap);
        launcherControl = new LauncherControl(hardwareMap);
        intakeControl = new IntakeControl(hardwareMap);
        sensorDisplay = new Sensors();
        sensorDisplay.SensorsINIT(hardwareMap);

        telemetry.addLine("Basic Autonomous Ready");
        telemetry.update();
        Parameters.coldStart = false;
        indexer.inBlock.setPosition(1);

        waitForStart();

        // Build and run trajectory to target pose

        Action main = drive.actionBuilder(RedFarParameters.startPose)
                .strafeTo(RedFarParameters.launchLocation)
                .turnTo(Math.toRadians(RedFarParameters.launchHeading))
                .stopAndAdd(new LaunchCycleAction(indexer,launcherControl, drive, RedFarParameters.launchRPM, this, intakeControl, sensorDisplay))
                .stopAndAdd(new SetAutoIndexEnabledAction(true))
                .splineToSplineHeading(RedFarParameters.firstGrab,Math.toRadians(90))

                // New: Slow drive forward for intake (adjust distance/speed)
                .lineToY(RedFarParameters.firstGrab.position.y + RedFarParameters.intakeForwardDistance,new TranslationalVelConstraint(RedFarParameters.intakeSpeed),new ProfileAccelConstraint(-5,5)
                )
                .stopAndAdd(new SetAutoIndexEnabledAction(false))
                // New: Return to launch and fire again
                .strafeTo(RedFarParameters.launchLocation)
                .turnTo(Math.toRadians(RedFarParameters.launchHeading))
                .stopAndAdd(new LaunchCycleAction(indexer,launcherControl, drive, RedFarParameters.launchRPM, this, intakeControl, sensorDisplay))  // Second launch cycle
                .splineToSplineHeading(RedFarParameters.firstGrab,Math.toRadians(90))
                .build();

        Actions.runBlocking(
                new RaceAction(
                        main,
                        new SubsystemUpdateAction(indexer, intakeControl, sensorDisplay, launcherControl, drive)

                )

        );


        // Keep updating until opmode stops
        while (opModeIsActive()) {
            indexer.update();

            drive.updatePoseEstimate();
            Parameters.startPose = drive.localizer.getPose();
            intakeControl.update(sensorDisplay);
            telemetry.update();
        }}}