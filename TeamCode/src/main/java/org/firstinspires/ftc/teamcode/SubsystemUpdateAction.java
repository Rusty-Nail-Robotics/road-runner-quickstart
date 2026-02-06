package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class SubsystemUpdateAction implements Action {

    private final DrumIndexer indexer;
    private final IntakeControl intake;
    private final Sensors sensorDisplay;
    private final LauncherControl launcher;
    private final MecanumDrive drive;


    private static final double BALL_DETECT_MM = 80;

    public SubsystemUpdateAction(DrumIndexer indexer,
                                 IntakeControl intake,
                                 Sensors sensorDisplay,
                                 LauncherControl launcher,
                                 MecanumDrive drive) {
        this.indexer = indexer;
        this.intake = intake;
        this.sensorDisplay = sensorDisplay;
        this.launcher = launcher;
        this.drive = drive;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Always service subsystems
        indexer.update();
        if (intake != null) intake.update(sensorDisplay);
        Parameters.startPose = drive.localizer.getPose();

        // DO NOT auto-index off the distance sensor while launching
        if (!BlueFarParameters.launching && BlueFarParameters.autoIndexEnabled) {

            if (indexer.DrumAtTarget() && Parameters.drum_in_out == 1) {

                if (sensorDisplay.GetDetectedPocketDistance() < BALL_DETECT_MM) {

                    int currentPocket = Parameters.pocketTarget;
                    switch (currentPocket){

                        case 0:
                            indexer.SetDrumPosition(2);
                            break;

                        case 2:
                            indexer.SetDrumPosition(4);
                            break;

                        case 4:
                            indexer.SetDrumPosition(5);
                            Parameters.launcherOn = true;
                            Parameters.drum_in_out = 0;
                            break;
                    }
                }
            }
        }


        return true;
    }
}
