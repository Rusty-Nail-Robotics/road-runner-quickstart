package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class SetAutoIndexEnabledAction implements Action {
    private final boolean enabled;
    private boolean done = false;

    public SetAutoIndexEnabledAction(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!done) {
            BlueFarParameters.autoIndexEnabled = enabled;
            done = true;
        }
        return false; // instant
    }
}
