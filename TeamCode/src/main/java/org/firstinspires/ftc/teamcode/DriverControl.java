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




        while(!opModeIsActive() && Parameters.telemetryOutput) {
            telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());
            pocketSensors.displayData(telemetry);
            telemetry.addData("DrumPosition = ", indexer.GetDrumPosition());
            telemetry.update();
        }

                waitForStart();

        while (opModeIsActive()) {



             if (gamepad1.left_bumper) {
               ;
             } else
                if (gamepad1.x) {
                indexer.startPush(); // Single push test
            }
            if(gamepad1.start){launcherOn = 1;}//launcherControl.setRPM(Parameters.farRPM);}
            if(gamepad1.back){launcherOn = 0;}//launcherControl.setRPM(0);}

            if(gamepad1.b){intakeControl.startReverse();}//test git



            if(launcherOn == 1){
                double currentLocationX = drive.localizer.getPose().position.x;
                if(currentLocationX < -30){launcherControl.setRPM(Parameters.farRPM);}
                if(currentLocationX > -30){launcherControl.setRPM(Parameters.closeRPM);}
            }else{
                launcherControl.setRPM(0);
            }
            // Update PIDF and push in loop (non-blocking)
            UpdateSystems();
            intakeMode();


            telemetry.addData("Indexer Target Pocket", Parameters.pocketTarget + "," + Parameters.drum_in_out);
            telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
            telemetry.addData("Target Position", indexer.targetPocket);
            telemetry.addData("Current Position", indexer.GetDrumPosition());
            telemetry.addData("Location XY Rot = ", drive.localizer.getPose().position.x);
            telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());


            telemetry.update();
        }
    }



    private void intakeMode(){
        if(indexer.DrumAtTarget()) {
            if(Parameters.drum_in_out == 1){

                if (pocketSensors.GetDetectedPocketDistance() < 70) {
                    int currentPocket = Parameters.pocketTarget;
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
                            intakeControl.startReverse();
                            break;

                    }
                }}}
    }

    private void RapidFire(){
            Parameters.launcherOn = true;
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
            if(Parameters.telemetryOutput) {
                telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
                telemetry.addData("Target Position", indexer.targetPosition);
                telemetry.addData("Current Position", indexer.GetDrumPosition());
                telemetry.update();
            }
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

        if (alignTimer.milliseconds() >= 3000) {
            telemetry.log().add("Alignment timeout - Check PID/encoder/mechanics");
        }
    }

    private void UpdateSystems(){
        indexer.update();
        pocketSensors.displayData(telemetry);
        intakeControl.update(pocketSensors);
        driveControl.update(gamepad1);
        drive.updatePoseEstimate();
        launcherControl.Update(drive);
    }
}