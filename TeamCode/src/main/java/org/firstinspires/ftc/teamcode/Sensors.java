package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor2;


    // Constructor: Initialize sensors with HardwareMap
    public void SensorsINIT(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor"); // REV 2m Distance Sensor name in config
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2"); // REV 2m Distance Sensor name in config

    }

    // Method to read and display sensor data (call in OpMode loop with telemetry)

    public double GetDetectedPocketDistance() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        double distance2 = distanceSensor2.getDistance(DistanceUnit.MM);

        return Math.max(distance, distance2);
    }

    public void displayData(Telemetry telemetry) {
        // Distance Sensor data
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        double distance2 = distanceSensor2.getDistance(DistanceUnit.MM);
        telemetry.addData("distance 1 = ", distance);
        telemetry.addData("distance 2 = ", distance2);
    }

}