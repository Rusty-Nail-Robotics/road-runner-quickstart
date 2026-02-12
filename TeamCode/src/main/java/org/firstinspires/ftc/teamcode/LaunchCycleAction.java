package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchCycleAction implements Action {

    private final DrumIndexer indexer;
    private final LauncherControl launcher;
    private final MecanumDrive drive;
    private final LinearOpMode opMode;
    private final IntakeControl intakeControl;
    private final Sensors pocketSensors;
    private double launchRPM;
    private ElapsedTime launchDelay;


    private int step = 1;
    private boolean started = false;

    private final ElapsedTime timer = new ElapsedTime();

    private enum Phase { ALIGNING, SPINUP, PUSHING }
    private Phase phase = Phase.ALIGNING;

    // Tune if needed
    private static final double ALIGN_TIMEOUT_S = 3.0;
    private static final double SPINUP_DELAY_S = 0.5; // 500ms for launcher spool-up
    private static final double PUSH_TIMEOUT_S = 2.0;

    public LaunchCycleAction(DrumIndexer indexer,
                             LauncherControl launcher,
                             MecanumDrive drive, double launchRPM, LinearOpMode opMode, IntakeControl intakeControl, Sensors pocketSensors) {
        this.indexer = indexer;
        this.launcher = launcher;
        this.drive = drive;
        this.launchRPM = launchRPM;
        this.opMode = opMode;
        launchDelay = new ElapsedTime();
        this.intakeControl = intakeControl;
        this.pocketSensors = pocketSensors;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        if (!started) {
            started = true;
            launchDelay.reset();
        }
        UpdateSystems();
        switch (step) {
            case 1:
                intakeControl.StopIntake();
                Parameters.launcherOn = true;
                launcher.setRPM(launchRPM);
                indexer.outBlock.setPosition(0);
                if (launchDelay.milliseconds() > Parameters.launchDelayMS) {
                    step = step + 1;
                }
                started = true;
                break;
            case 2:
                alignAndPush(5);
                started = true;
                step = step + 1;
                break;
            case 3:
                alignAndPush(3);
                started = true;
                step = step + 1;
                break;
            case 4:
                alignAndPush(1);
                started = true;
                step = step + 1;
                break;

            case 5:
                indexer.SetDrumPosition(0);
                Parameters.drum_in_out = 1;
                indexer.outBlock.setPosition(1);
                indexer.inBlock.setPosition(0);
                Parameters.launcherOn = false;
                launcher.setRPM(0);
                Parameters.autoIndexEnabled = true;
                started = false;
                step = 1;
                break;

        }

        return started;


    }




    // Helper for rapid sequence (waits for push complete before next index)
    private void alignAndPush(int pocketPosition) {
        indexer.outBlock.setPosition(0);
        indexer.SetDrumPosition(pocketPosition);

        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        while (opMode.opModeIsActive() && alignTimer.milliseconds() < 3000) { // 2s timeout for alignment (tune)
            UpdateSystems();

            if (indexer.DrumAtTarget()) { // Settled
                indexer.startPush();

                ElapsedTime pushTimer = new ElapsedTime();
                pushTimer.reset();
                while (opMode.opModeIsActive() && !indexer.isPushComplete() && pushTimer.milliseconds() < 2000) { // 1s timeout for push (tune)
                    UpdateSystems();

                }
                indexer.outBlock.setPosition(1);
                break;
            }
        }

    }

        private void UpdateSystems(){
            indexer.update();
            //TelemetryOutput();
            intakeControl.update(pocketSensors);
            //driveControl.update(gamepad1);
            //drive.updatePoseEstimate();
            //launcherControl.Update();
            //Unjam();

        }
}