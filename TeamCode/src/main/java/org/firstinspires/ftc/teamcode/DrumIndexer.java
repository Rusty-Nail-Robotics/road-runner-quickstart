package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DrumIndexer {
    //UltraPlanetary 1:1 == 28 cpr, Worm == 28:1,  Total CPR ==784, half rot == 392

    private int pocketLocation0 = 0;//intake pocket 1
    private int pocketLocation1 = 131;//Output pocket 2
    private int pocketLocation2 = 261;//intake pocket 3
    private int pocketLocation3 = 392;//output Pocket 1
    private int pocketLocation4 = 522;//intake pocket 2
    private int pocketLocation5 = 653;//output pocket 3
    public int[] pocketLocationArray = {pocketLocation0, pocketLocation1, pocketLocation2, pocketLocation3, pocketLocation4, pocketLocation5};

    public int targetPocket = 0;



    private DcMotorEx drum;
    private Servo pusher; // Optional, if needed for pushing
    public Servo outBlock;
    private Servo inBlock;
    private PIDFController drumPIDF;

    private LinearOpMode opMode;
    public int targetPosition = 0;
    private int drumTargetTolerance = 25;
    private int drumTargetVelocityTolerance = 100;

    // Non-blocking push state
    private boolean pushing = false;
    private long pushStartTime = 0;
    private static final long EXTEND_HOLD_MS = 300; // Time to hold extended (tune for push)
    private static final long RETRACT_DELAY_MS = 500; // Delay before retracting (added per request)

    private boolean retracting = false;
    private long retractStartTime = 0;


    // Constructor: Initialize with HardwareMap
    public void DrumIndexerInit(HardwareMap hardwareMap) {
        drum = hardwareMap.get(DcMotorEx.class, "indexDrum");
        pusher = hardwareMap.get(Servo.class, "pusher");
        outBlock = hardwareMap.get(Servo.class, "outBlock");
        inBlock = hardwareMap.get(Servo.class, "inBlock");


        drum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(Parameters.coldStart){drum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        drum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // needed for velocity readout

        // Initialize PIDF with your values
        drumPIDF = new PIDFController(Parameters.drumP, Parameters.drumI, Parameters.drumD, Parameters.drumF);


        pusher.setPosition(0.0); // Retracted position
        outBlock.setPosition(1);
        inBlock.setPosition(0.0); // Assume 0.0 is open (allow intake); tune if reversed
    }

    public void update() {
        double currentPosition = GetDrumPosition();
        double outputPower = drumPIDF.calculate(currentPosition, targetPosition);
        drum.setPower(outputPower);

        if (pushing) {
            if (System.currentTimeMillis() - pushStartTime >= EXTEND_HOLD_MS + RETRACT_DELAY_MS) {
                //outBlock.setPosition(1);
                pusher.setPosition(0.0); // Retract after delay
                retractStartTime = System.currentTimeMillis();
                pushing = false;
                retracting = true;
            }
        } else if (retracting) {
            if (System.currentTimeMillis() - retractStartTime >= 1000) { // Brief retract settle
                retracting = false;
            }
        }

    }
    public void SetDrumPosition(int drumPocketTarget) {
        targetPocket = drumPocketTarget;
        targetPosition = pocketLocationArray[drumPocketTarget];

    }




    public void DrumMove(int pocketsToMove){
        int steps = Math.abs(pocketsToMove);
        int direction = (pocketsToMove >=0) ? 1 : -1;
        for (int i = 0; i < steps; i++){
            targetPocket = targetPocket + direction;
            if(targetPocket > 6){
                targetPocket = 1;
            } else if (targetPocket < 0) {
                targetPocket = 6;
            }
        }

        targetPosition = pocketLocationArray[targetPocket];
    }

    public int GetClosestPocket(){
        int closestPocket;
        int currentPosition = GetDrumPosition();
        int pocket0_1Halfway = (pocketLocationArray[1]-pocketLocationArray[0])/2;
        int pocket1_2Halfway = (pocketLocationArray[2]-pocketLocationArray[1])/2;
        int pocket2_3Halfway = (pocketLocationArray[3]-pocketLocationArray[2])/2;
        int pocket3_4Halfway = (pocketLocationArray[4]-pocketLocationArray[3])/2;
        int pocket4_5Halfway = (pocketLocationArray[5]-pocketLocationArray[4])/2;
        int pocket5_6Halfway = (pocketLocationArray[6]-pocketLocationArray[5])/2;
        if(currentPosition <=  pocket0_1Halfway){
            closestPocket = 0;
        } else if (currentPosition <= pocket1_2Halfway && currentPosition > pocket0_1Halfway) {
            closestPocket = 1;
        } else if (currentPosition <= pocket2_3Halfway && currentPosition > pocket1_2Halfway) {
            closestPocket = 2;
        } else if (currentPosition <= pocket3_4Halfway && currentPosition > pocket2_3Halfway) {
            closestPocket = 3;
        } else if (currentPosition <= pocket4_5Halfway && currentPosition > pocket5_6Halfway) {
            closestPocket = 4;
        } else if (currentPosition <= pocket5_6Halfway && currentPosition > pocket4_5Halfway) {
            closestPocket = 5;
        } else if (currentPosition > pocket5_6Halfway) {
            closestPocket = 6;
        }else{
            closestPocket = 0;
        }

        return closestPocket;
    }

    public int GetDrumPosition(){
        return drum.getCurrentPosition();
    }

    public double GetDrumVelocity(){
        return drum.getVelocity();
    }

    public boolean DrumAtTarget(){
        if (Math.abs(GetDrumVelocity()) <= drumTargetVelocityTolerance && Math.abs(GetDrumPosition() - targetPosition) <= drumTargetTolerance){
            return true;
        }else{
            return false;
        }
    }

    public boolean isPushComplete() {
        return !pushing && !retracting;
    }

    public void startPush() {
        if (!pushing && !retracting) {
            //outBlock.setPosition(.5);
            pusher.setPosition(1.0); // Extend to push
            pushStartTime = System.currentTimeMillis();
            pushing = true;
        }
    }




}