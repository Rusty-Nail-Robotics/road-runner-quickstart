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
        launcher.Update(drive);
        intake.update(sensorDisplay);


        // DO NOT auto-index off the distance sensor while launching
        if (Parameters.autoIndexEnabled) {
            indexer.inBlock.setPosition(0);
                intakeMode();
                }else{indexer.inBlock.setPosition(1);}




        return true;
    }
    private void intakeMode(){
        if(indexer.DrumAtTarget()) {
            if(Parameters.drum_in_out == 1){

                if (sensorDisplay.GetDetectedPocketDistance() < 70) {
                    int currentPocket = indexer.targetPocket;
                    switch (currentPocket){

                        case 0:
                            indexer.SetDrumPosition(2);
                            break;

                        case 2:
                            indexer.SetDrumPosition(4);
                            break;

                        case 4:
                            indexer.SetDrumPosition(5);
                            Parameters.autoIndexEnabled = false;
                            //launcherOn = 1;
                            //launcherControl.setRPM(Parameters.farRPM);
                            //intakeControl.startReverse();

                            //RapidFire();
                            break;

                    }
                }}}
    }
}
