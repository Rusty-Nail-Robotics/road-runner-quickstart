package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorDisplay {

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    // Constructor: Initialize sensors with HardwareMap
    public SensorDisplay(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor"); // REV 2m Distance Sensor name in config
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); // REV Color Sensor name in config
    }

    // Method to read and display sensor data (call in OpMode loop with telemetry)

    public double GetDetectedDistance(){
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
    public void displayData(Telemetry telemetry) {
        // Distance Sensor data
        double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
       // telemetry.addData("Distance (cm)", "%.2f", distanceCm); // e.g., <5cm for ball in pocket

        // Color Sensor data
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha(); // Transparency/light level
       // telemetry.addData("Color RGB A", "%d %d %d %d", red, green, blue, alpha);

        // Detected color
        String detected = getDetectedColor();
        //telemetry.addData("Detected", detected); // "Purple", "Green", or "Nothing"

        // Optional: Hybrid check (distance + color for reliability)
        boolean possibleBall = distanceCm < 5.0 && !detected.equals("Nothing");
       // telemetry.addData("Possible Ball in Pocket", possibleBall ? "Yes" : "No");
    }

    // Method to detect color: "Purple", "Green", or "Nothing" (tuned based on provided purple values: R=934 G=1074 B=1560 A=1188)
    public String getDetectedColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int intensity = red + green + blue; // Overall brightness

        if (intensity < 1500) { // Tune: Low for "nothing" (empty pocket; assuming lower values when no ball)
            return "Nothing";
        }

        // Purple: High blue (dominant), medium red/green (based on sample: B high ~1560, R/G ~900-1100)
        if (blue > 1200 && red > 800 && green > 800 && blue > red && blue > green && (red + green) < 1.5 * blue) {
            return "Purple";
        }

        // Green: High green, lower red/blue (tune based on green ball sample; assume G dominant ~1500+, R/B <1000)
        if (green > 1200 && red < 1000 && blue < 1000 && green > 1.2 * (red + blue)) {
            return "Green";
        }

        return "Nothing"; // Fallback
    }
}