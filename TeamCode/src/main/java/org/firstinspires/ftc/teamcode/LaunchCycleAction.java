package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchCycleAction implements Action {

    private final DrumIndexer indexer;
    private final LauncherControl launcher;
    private final MecanumDrive drive;
    private double launchRPM;

    private int step = 0;
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
                             MecanumDrive drive, double launchRPM) {
        this.indexer = indexer;
        this.launcher = launcher;
        this.drive = drive;
        this.launchRPM = launchRPM;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        if (!started) {
            started = true;
            BlueFarParameters.launching = true; // LOCKOUT ON

            // RPM selection (adjust sign based on your field coords)
            launcher.setRPM(launchRPM);

            // Start with pocket ONE alignment
            startAlign(5);
        }

        if(Parameters.telemetryOutput) {
            packet.put("launchStep", step);
            packet.put("launchPhase", phase.toString());
            packet.put("launching", BlueFarParameters.launching);
        }
        switch (step) {
            case 0: // Pocket ONE OUT
                if (runAlignSpinupAndPush()) {
                    step++;
                    startAlign(5);
                }
                return true;

            case 1: // Pocket TWO OUT
                if (runAlignAndPush()) {
                    step++;
                    startAlign(3);
                }
                return true;

            case 2: // Pocket THREE OUT
                if (runAlignAndPush()) {
                    // Reset to pocket ONE IN
                    indexer.SetDrumPosition(0);

                    // Stop launcher
                    launcher.setRPM(0);

                    BlueFarParameters.launching = false; // LOCKOUT OFF
                    step++;
                }
                return true;

            default:
                // Finished
                BlueFarParameters.launching = false;
                return false;
        }
    }

    private void startAlign(int pocket) {
        phase = Phase.ALIGNING;
        timer.reset();
        indexer.SetDrumPosition(pocket);
    }

    // For first pocket: align -> spinup delay -> push
    private boolean runAlignSpinupAndPush() {
        if (phase == Phase.ALIGNING) {
            // Alignment timeout safety
            if (timer.seconds() > ALIGN_TIMEOUT_S) {
                phase = Phase.SPINUP;
                timer.reset();
                return false;
            }

            if (indexer.DrumAtTarget()) {
                phase = Phase.SPINUP;
                timer.reset();
            }
            return false;
        }

        if (phase == Phase.SPINUP) {
            if (timer.seconds() >= SPINUP_DELAY_S) {
                phase = Phase.PUSHING;
                timer.reset();
                indexer.startPush();
            }
            return false;
        }

        // PUSHING phase
        if (timer.seconds() > PUSH_TIMEOUT_S) {
            phase = Phase.ALIGNING; // Reset for next pocket
            return true;
        }

        if (indexer.isPushComplete()) {
            phase = Phase.ALIGNING; // Reset for next pocket
            return true;
        }

        return false;
    }

    // For subsequent pockets: align -> push (no spinup)
    private boolean runAlignAndPush() {
        if (phase == Phase.ALIGNING) {
            // Alignment timeout safety
            if (timer.seconds() > ALIGN_TIMEOUT_S) {
                phase = Phase.PUSHING;
                timer.reset();
                indexer.startPush();
                return false;
            }

            if (indexer.DrumAtTarget()) {
                phase = Phase.PUSHING;
                timer.reset();
                indexer.startPush();
            }
            return false;
        }

        // PUSHING phase
        if (timer.seconds() > PUSH_TIMEOUT_S) {
            phase = Phase.ALIGNING; // Reset for next pocket
            return true;
        }

        if (indexer.isPushComplete()) {
            phase = Phase.ALIGNING; // Reset for next pocket
            return true;
        }

        return false;
    }
}