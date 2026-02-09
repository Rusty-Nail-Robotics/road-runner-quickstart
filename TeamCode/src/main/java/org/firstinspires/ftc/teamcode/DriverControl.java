package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Control", group = "Test")
public class DriverControl extends LinearOpMode {

    private DrumIndexer indexer;
    private Sensors pocketSensors;
    private LauncherControl launcherControl;
    private IntakeControl intakeControl;
    private DriverMecanum driveControl;
    private MecanumDrive drive;
    private ElapsedTime launchDelay;
    private ElapsedTime telemetryTimer;
    private boolean leftBumperLastState = false;
    private boolean rightBumperLastState = false;
    private boolean xLastState = false;

    Pose2d beginPose = Parameters.startPose;
    Pose2d currentPose = new Pose2d(0,0,0);
    private int launcherOn = 0;
    @Override
    public void runOpMode() {
        // Initialize indexer
        drive = new MecanumDrive(hardwareMap, beginPose);
        indexer = new DrumIndexer();
        indexer.DrumIndexerInit(hardwareMap);
        pocketSensors = new Sensors();
        pocketSensors.SensorsINIT(hardwareMap);
        launcherControl = new LauncherControl(hardwareMap);
        intakeControl = new IntakeControl(hardwareMap);
        driveControl = new DriverMecanum(hardwareMap);
        launchDelay = new ElapsedTime();
        telemetryTimer = new ElapsedTime();
        if(!Parameters.coldStart){indexer.SetDrumOffset(Parameters.drumLastPosition);}





        while(!opModeIsActive() && Parameters.telemetryOutput) {
            telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());
            pocketSensors.displayData(telemetry);
            telemetry.addData("DrumPosition = ", indexer.GetDrumPosition());
            telemetry.update();
        }

                waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x && !xLastState) {
                if(Parameters.launcherHigh){
                    Parameters.launcherHigh = false;
                    launcherControl.highLowIndicator1R.setState(true);
                    launcherControl.highLowIndicator1G.setState(false);
                }else{
                    Parameters.launcherHigh = true;
                    launcherControl.highLowIndicator1R.setState(false);
                    launcherControl.highLowIndicator1G.setState(true);
                }
                xLastState = true;
            }else if(!gamepad1.x){
                xLastState = false;
            }


            if(gamepad1.y){
                indexer.SetDrumPosition(0);
                if(gamepad1.left_bumper && !leftBumperLastState){
                    indexer.SetDrumOffset(5);
                    leftBumperLastState = true;
                }else if(!gamepad1.left_bumper){
                    leftBumperLastState = false;
                }
                if(gamepad1.right_bumper && !rightBumperLastState){
                    indexer.SetDrumOffset(-5);
                    rightBumperLastState = true;
                }else if (!gamepad1.right_bumper){
                    rightBumperLastState = false;
                }
            }


             if (gamepad1.left_bumper) {
               RapidFire();
             }
            if(gamepad1.start){Parameters.launcherOn  = true;}
            if(gamepad1.back){Parameters.launcherOn = false;}//launcherControl.setRPM(0);}

            if(gamepad1.b){intakeControl.startReverse();}//test git





            UpdateSystems();
            intakeMode();


        }
    }



    private void intakeMode(){
        if(indexer.DrumAtTarget()) {
            if(Parameters.drum_in_out == 1){

                if (pocketSensors.GetDetectedPocketDistance() < 70) {
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
                            //launcherOn = 1;
                            //launcherControl.setRPM(Parameters.farRPM);
                            //intakeControl.startReverse();

                            //RapidFire();
                            break;

                    }
                }}}
    }

    private void RapidFire(){
        intakeControl.StopIntake();
            Parameters.launcherOn = true;
            indexer.outBlock.setPosition(0);
            launchDelay.reset();
            while(launchDelay.milliseconds() < Parameters.launchDelayMS){
                UpdateSystems();
            }
            alignAndPush(5);
            alignAndPush(3);
            alignAndPush(1);
            indexer.SetDrumPosition(0);
            Parameters.drum_in_out = 1;
            indexer.outBlock.setPosition(1);
            Parameters.launcherOn = false;
        }



    // Helper for rapid sequence (waits for push complete before next index)
    private void alignAndPush(int pocketPosition) {
        indexer.SetDrumPosition(pocketPosition);

        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        while (opModeIsActive() && alignTimer.milliseconds() < 3000) { // 2s timeout for alignment (tune)
            UpdateSystems();

            if (indexer.DrumAtTarget()) { // Settled
                indexer.startPush();

                ElapsedTime pushTimer = new ElapsedTime();
                pushTimer.reset();
                while (opModeIsActive() && !indexer.isPushComplete() && pushTimer.milliseconds() < 2000) { // 1s timeout for push (tune)
                    UpdateSystems();
                }
                break;
            }
        }


    }

    private void UpdateSystems(){
        indexer.update();
        TelemetryOutput();
        intakeControl.update(pocketSensors);
        driveControl.update(gamepad1);
        drive.updatePoseEstimate();
        launcherControl.Update(drive);
        //Unjam();

    }

    private void Unjam(){
        if(gamepad1.x && !xLastState){
            Parameters.launcherOn = false;
            indexer.ClearJam();
            xLastState = true;
        }else if(!gamepad1.x && xLastState){
            xLastState = false;

        }
        if(gamepad1.right_bumper){
            Parameters.launcherOn = false;
            indexer.SetDrumPosition(0);
            Parameters.drum_in_out = 1;
        }
    }

    private void TelemetryOutput(){
        if(Parameters.telemetryOutput) {
            if (telemetryTimer.milliseconds() > Parameters.telemetryIntervalMs) {
                pocketSensors.displayData(telemetry);
                telemetry.addData("Indexer Target Pocket", indexer.targetPocket + "," + Parameters.drum_in_out);
                telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
                telemetry.addData("Target Position", indexer.targetPosition);
                telemetry.addData("Current Position", indexer.GetDrumPosition());
                telemetry.addData("Location XY Rot = ", drive.localizer.getPose().position.x);
                telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());
                telemetry.addData("target velocity = ", LauncherControl.ticksPerSec);
                telemetry.addData("left RPM = ", launcherControl.launcherLeft.getVelocity());
                telemetry.addData("left Power = ", launcherControl.launcherLeft.getPower());
                telemetry.addData("right RPM = ", launcherControl.launcherRight.getVelocity());
                telemetry.addData("right Power = ", launcherControl.launcherRight.getPower());


                telemetry.update();
                telemetryTimer.reset();
            }
        }
    }
}