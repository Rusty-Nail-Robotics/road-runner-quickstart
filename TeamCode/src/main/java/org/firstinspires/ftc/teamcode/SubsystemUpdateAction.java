package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class SubsystemUpdateAction implements Action {

    private final DrumIndexer indexer;
    private final IntakeControl intake;
    private final SensorDisplay sensorDisplay;
    private final LauncherControl launcher;
    private final MecanumDrive drive;


    private static final double BALL_DETECT_MM = 80;

    public SubsystemUpdateAction(DrumIndexer indexer,
                                 IntakeControl intake,
                                 SensorDisplay sensorDisplay,
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
        indexer.update(sensorDisplay);
        indexer.updatePush();
        if (intake != null) intake.update(sensorDisplay);
        Parameters.startPose = drive.localizer.getPose();

        // DO NOT auto-index off the distance sensor while launching
        if (!BlueFarParameters.launching && BlueFarParameters.autoIndexEnabled) {
//indexer.inBlock.setPosition(.2);
            Parameters.drum_in_out=1;
            if (indexer.DrumAtTarget() && Parameters.drum_in_out == 1) {

                if (sensorDisplay.GetDetectedDistance() < BALL_DETECT_MM) {

                    int currentPocket = Parameters.pocketTarget;
                    switch (currentPocket) {
                        case 1:
                            indexer.setAlignment(DrumIndexer.Pocket.TWO, DrumIndexer.Port.IN);
                            break;

                        case 2:
                            indexer.setAlignment(DrumIndexer.Pocket.THREE, DrumIndexer.Port.IN);
                            break;

                        case 3:
                            indexer.setAlignment(DrumIndexer.Pocket.ONE, DrumIndexer.Port.OUT);
                            packet.put("launching", BlueFarParameters.launching);
                            double x = drive.localizer.getPose().position.x;
                            if (x > 30) {launcher.setRPM(Parameters.farRPM);}
                            else         {launcher.setRPM(Parameters.closeRPM);}
                            //indexer.inBlock.setPosition(1.0);
                            Parameters.drum_in_out=0;
                            break;
                    }
                }
            }
        }


        return true;
    }
}
