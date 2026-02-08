package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DrumIndexer {
    //UltraPlanetary 1:1 == 28 cpr, Worm == 28:1,  Total CPR ==784, half rot == 392


    public int[] pocketLocationArray = {Parameters.pocketLocation0, Parameters.pocketLocation1, Parameters.pocketLocation2, Parameters.pocketLocation3, Parameters.pocketLocation4, Parameters.pocketLocation5};

    public int targetPocket = 0;



    private DcMotorEx drum;
    private Servo pusher; // Optional, if needed for pushing
    public Servo outBlock;
    private Servo inBlock;
    //public PIDFController drumPIDF;
    public PIDcontrol drumPIDF;

    private LinearOpMode opMode;
    public int targetPosition = 0;
    private int drumTargetTolerance = 20;
    private int drumTargetVelocityTolerance = 50;

    // Non-blocking push state
    private boolean pushing = false;
    private long pushStartTime = 0;
    private static final long EXTEND_HOLD_MS = 250; // Time to hold extended (tune for push)
    private static final long RETRACT_DELAY_MS = 250; // Delay before retracting (added per request)

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
        //drum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize PIDF with your values
        //drumPIDF = new PIDFController(Parameters.drumP, Parameters.drumI, Parameters.drumD, Parameters.drumF);
        drumPIDF = new PIDcontrol();
        drumPIDF.SetPIDCoeficients(Parameters.drumP, Parameters.drumI, Parameters.drumD);
        //drumPIDF.setTolerance(3);


        pusher.setPosition(0.0); // Retracted position
        outBlock.setPosition(1);
        inBlock.setPosition(0.0); // Assume 0.0 is open (allow intake); tune if reversed
    }

    public void update() {
        Parameters.drumLastPosition = GetDrumPosition();
        targetPosition = pocketLocationArray[targetPocket];
        //if(!drumPIDF.atSetPoint()) {
        drumPIDF.SetPIDCoeficients(Parameters.drumP, Parameters.drumI, Parameters.drumD);
            double currentPosition = GetDrumPosition();
            //double outputPower = drumPIDF.calculate(currentPosition, targetPosition);
        double outputPower = drumPIDF.PIDControl(targetPosition, currentPosition);
            drum.setPower(outputPower);
        //}else{drum.setPower(0);}


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
        targetPosition = pocketLocationArray[targetPocket];

    }




    public void DrumMove(int pocketsToMove){
        int steps = Math.abs(pocketsToMove);
        int direction = (pocketsToMove >=0) ? 1 : -1;
        for (int i = 0; i < steps; i++){
            targetPocket = targetPocket + direction;
            if(targetPocket > 5){
                targetPocket = 0;
            } else if (targetPocket < 0) {
                targetPocket = 5;
            }

        }

        targetPosition = pocketLocationArray[targetPocket];
    }

    public int GetClosestPocket(){
        int closestPocket;
        int currentPosition = GetDrumPosition();
        int pocket0_1Halfway = (pocketLocationArray[1]+pocketLocationArray[0])/2;
        int pocket1_2Halfway = (pocketLocationArray[2]+pocketLocationArray[1])/2;
        int pocket2_3Halfway = (pocketLocationArray[3]+pocketLocationArray[2])/2;
        int pocket3_4Halfway = (pocketLocationArray[4]+pocketLocationArray[3])/2;
        int pocket4_5Halfway = (pocketLocationArray[5]+pocketLocationArray[4])/2;
        if(currentPosition <=  pocket0_1Halfway){
            closestPocket = 0;
        } else if ((currentPosition <= pocket1_2Halfway) && (currentPosition > pocket0_1Halfway)) {
            closestPocket = 1;
        } else if ((currentPosition <= pocket2_3Halfway) && (currentPosition > pocket1_2Halfway)) {
            closestPocket = 2;
        } else if ((currentPosition <= pocket3_4Halfway) && (currentPosition > pocket2_3Halfway)) {
            closestPocket = 3;
        } else if ((currentPosition <= pocket4_5Halfway) && (currentPosition > pocket3_4Halfway)) {
            closestPocket = 4;
        } else if (currentPosition > pocket4_5Halfway) {
            closestPocket = 5;
        }else{
            closestPocket = 0;
        }

        return closestPocket;
    }

    public void ClearJam(){
        targetPosition = pocketLocationArray[GetClosestPocket()];
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

    public void SetDrumOffset(int offset){
        for(int i = 0; i < pocketLocationArray.length; i++){
            pocketLocationArray[1] = pocketLocationArray[i] - offset;
        }
    }



}