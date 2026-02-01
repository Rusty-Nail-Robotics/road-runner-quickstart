package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Control", group = "Test")
public class DriverControl extends LinearOpMode {

    private DrumIndexer indexer;
    private SensorDisplay sensorDisplay;
    private LauncherControl launcherControl;
    private IntakeControl intakeControl;
    private DriverMecanum driveControl;
    Pose2d beginPose = Parameters.startPose;
    Pose2d currentPose = new Pose2d(0,0,0);
    private int launcherOn = 0;
    private int launcherHigh = 0;
    private int launcherHighButtonLastState = 0;
    private int resetButtonLastState = 0;
    @Override
    public void runOpMode() {
        // Initialize indexer
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        indexer = new DrumIndexer(hardwareMap);
        sensorDisplay = new SensorDisplay(hardwareMap);
        launcherControl = new LauncherControl(hardwareMap);
        intakeControl = new IntakeControl(hardwareMap);
        driveControl = new DriverMecanum(hardwareMap);
        Parameters.drum_in_out = 1;




        while(!opModeIsActive()){
            telemetry.addData("distance = ", sensorDisplay.GetDetectedDistance());
            sensorDisplay.displayData(telemetry);
            telemetry.addData("DrumPosition = ", indexer.getCurrentPosition());
            telemetry.update();
        }

                waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.y && resetButtonLastState == 0){
                if (Parameters.correction == 0){Parameters.correction = Parameters.IN_TO_OUT_OFFSET;}
                else{Parameters.correction = 0;}
                indexer.setAlignment(DrumIndexer.Pocket.ONE, DrumIndexer.Port.IN);
                resetButtonLastState = 1;
            }
            else{
                resetButtonLastState = 0;
            }

            if(gamepad1.x && launcherHighButtonLastState == 0){
               if(launcherHigh == 0){launcherHigh = 1;}
               else{launcherHigh = 0;}
                launcherHighButtonLastState = 1;
            }
            else{
                launcherHighButtonLastState = 0;
            }

             if (gamepad1.left_bumper) {
               //  Rapid output: Align each to OUT and push
                 indexer.outBlock.setPosition(.5);
                 launcherOn = 1;
                        if (launcherHigh == 1 ){launcherControl.setRPM(Parameters.farRPM);}
                        else{launcherControl.setRPM(Parameters.closeRPM);}

                     //double currentLocationX = drive.localizer.getPose().position.x;
                     //if(currentLocationX < -30){launcherControl.setRPM(Parameters.farRPM);}
                     //if(currentLocationX > -30){launcherControl.setRPM(Parameters.closeRPM);}

                 // launcherControl.setRPM(Parameters.farRPM);
                alignAndPush(DrumIndexer.Pocket.ONE, DrumIndexer.Port.OUT);
                alignAndPush(DrumIndexer.Pocket.TWO, DrumIndexer.Port.OUT);
                alignAndPush(DrumIndexer.Pocket.THREE, DrumIndexer.Port.OUT);
                indexer.setAlignment(DrumIndexer.Pocket.ONE, DrumIndexer.Port.IN);
                //launcherControl.setRPM(0);
                 launcherOn = 0;
                 launcherHigh = 0;
                 indexer.outBlock.setPosition(1);
           }// else
               // if (gamepad1.x) {
              //  indexer.startPush(); // Single push test
           // }
            if(gamepad1.start){launcherOn = 1;}//launcherControl.setRPM(Parameters.farRPM);}
            if(gamepad1.back){launcherOn = 0;}//launcherControl.setRPM(0);}

            if(gamepad1.b){intakeControl.startReverse();}//test git

            if(indexer.DrumAtTarget()) {
                if(Parameters.drum_in_out == 1){
                //if(sensorDisplay.GetDetectedDistance() < 180){Parameters.intakeManual = 1;}else{Parameters.intakeManual = 0;}
                if (sensorDisplay.GetDetectedDistance() < 70) {
                    int currentPocket = Parameters.pocketTarget;
                    switch (currentPocket){

                        case 1:
                            indexer.setAlignment(DrumIndexer.Pocket.TWO, DrumIndexer.Port.IN);
                            break;

                        case 2:
                            indexer.setAlignment(DrumIndexer.Pocket.THREE, DrumIndexer.Port.IN);
                            break;

                        case 3:
                            indexer.setAlignment(DrumIndexer.Pocket.ONE, DrumIndexer.Port.OUT);
                            launcherOn = 1;
                            //launcherControl.setRPM(Parameters.farRPM);
                            intakeControl.startReverse();
                        break;

                }
            }}}

            if(launcherOn == 1){
                if (launcherHigh == 1 ){launcherControl.setRPM(Parameters.farRPM);}
                else{launcherControl.setRPM(Parameters.closeRPM);}
//                double currentLocationX = drive.localizer.getPose().position.x;
  //              if(currentLocationX < -30){launcherControl.setRPM(Parameters.farRPM);}
    //            if(currentLocationX > -30){launcherControl.setRPM(Parameters.closeRPM);}
            }else{
                launcherControl.setRPM(0);
            }
            // Update PIDF and push in loop (non-blocking)
            indexer.update(sensorDisplay);
            indexer.updatePush();
            sensorDisplay.displayData(telemetry);
            intakeControl.update(sensorDisplay);
            driveControl.update(gamepad1);
            drive.updatePoseEstimate();


            telemetry.addData("Indexer Target Pocket", Parameters.pocketTarget + "," + Parameters.drum_in_out);
            telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
            telemetry.addData("Target Position", indexer.getTargetPosition());
            telemetry.addData("Current Position", indexer.getCurrentPosition());
            telemetry.addData("Location XY Rot = ", drive.localizer.getPose().position.x);
            telemetry.addData("distance = ", sensorDisplay.GetDetectedDistance());


            telemetry.update();
        }
    }

    // Helper for rapid sequence (waits for push complete before next index)
    private void alignAndPush(DrumIndexer.Pocket pocket, DrumIndexer.Port port) {
        indexer.setAlignment(pocket, port);

        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        while (opModeIsActive() && alignTimer.milliseconds() < 3000) { // 2s timeout for alignment (tune)
            indexer.update(sensorDisplay);
            indexer.updatePush();
            intakeControl.update(sensorDisplay);
            driveControl.update(gamepad1);
            telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
            telemetry.addData("Target Position", indexer.getTargetPosition());
            telemetry.addData("Current Position", indexer.getCurrentPosition());
            telemetry.update();
            if (indexer.DrumAtTarget()) { // Settled
                indexer.startPush();

                ElapsedTime pushTimer = new ElapsedTime();
                pushTimer.reset();
                while (opModeIsActive() && !indexer.isPushComplete() && pushTimer.milliseconds() < 2000) { // 1s timeout for push (tune)
                    indexer.update(sensorDisplay);
                    indexer.updatePush();
                    intakeControl.update(sensorDisplay);
                    driveControl.update(gamepad1);
                }
                break;
            }
        }

        if (alignTimer.milliseconds() >= 3000) {
            telemetry.log().add("Alignment timeout - Check PID/encoder/mechanics");
        }
    }
}