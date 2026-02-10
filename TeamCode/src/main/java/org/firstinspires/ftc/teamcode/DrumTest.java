package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drum Test", group = "Test")
public class DrumTest extends LinearOpMode {

    private DrumIndexer indexer;
    private Sensors pocketSensors;
    private LauncherControl launcherControl;
    private IntakeControl intakeControl;
    private DriverMecanum driveControl;
    private MecanumDrive drive;
    private ElapsedTime timer;
    Pose2d beginPose = Parameters.startPose;
    Pose2d currentPose = new Pose2d(0,0,0);
    private boolean leftBumperLastState = false;
    private boolean rightBumperLastState = false;
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
        timer = new ElapsedTime();




        while(!opModeIsActive() && Parameters.telemetryOutput) {
            telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());
            pocketSensors.displayData(telemetry);
            telemetry.addData("DrumPosition = ", indexer.GetDrumPosition());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            //indexer.targetPosition = Parameters.drumtesttarget;
           //indexer.drumPIDF = new PIDFController(Parameters.drumP, Parameters.drumI, Parameters.drumD, Parameters.drumF);

            indexer.drumPIDF.SetPIDCoeficients(Parameters.drumP, Parameters.drumI, Parameters.drumD);

            if(gamepad1.left_bumper && !leftBumperLastState){
                indexer.DrumMove(1);
                leftBumperLastState = true;
                Parameters.drumtesttarget = indexer.targetPocket;
            }else if(!gamepad1.left_bumper){
                leftBumperLastState = false;
            }
            if(gamepad1.right_bumper && !rightBumperLastState){
                indexer.DrumMove(-1);
                rightBumperLastState = true;
                Parameters.drumtesttarget = indexer.targetPocket;
            }else if (!gamepad1.right_bumper){
                rightBumperLastState = false;
            }
            if(!gamepad1.left_bumper && !gamepad1.right_bumper)
                indexer.SetDrumPosition(Parameters.drumtesttarget);


            while(!gamepad1.a && opModeIsActive()){
                if(timer.seconds() > 3){
                    int currentPocket = indexer.targetPocket;
                    switch (currentPocket) {
                        case 0:
                            indexer.SetDrumPosition(2);
                            break;
                        case 2:
                            indexer.SetDrumPosition(4);
                            break;
                        case 4:
                            indexer.SetDrumPosition(5);
                            break;
                        case 5:
                            indexer.SetDrumPosition(3);
                            break;
                        case 3:
                            indexer.SetDrumPosition(1);
                            break;
                        case 1:
                            indexer.SetDrumPosition(0);
                            break;
                    }
                    //indexer.DrumMove(1);
                    timer.reset();
                }
                UpdateSystems();
            }
            //indexer.targetPosition = (int) (indexer.targetPosition + (gamepad1.left_stick_x * 10));







            // Update PIDF and push in loop (non-blocking)
            UpdateSystems();
            //intakeMode();



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
        //driveControl.update(gamepad1);
        drive.updatePoseEstimate();
        launcherControl.Update(drive);
        ///telemetry.addData("Indexer Target Pocket", Parameters.pocketTarget + "," + Parameters.drum_in_out);
        telemetry.addData("Indexer At Target = ", indexer.DrumAtTarget() ? "yes" : "no");
        telemetry.addData("Target Position", indexer.targetPosition);
        telemetry.addData("Current Position", indexer.GetDrumPosition());
        telemetry.addData("drum Error = ", indexer.targetPosition-indexer.GetDrumPosition());
        telemetry.addData("Closest Pocket = ", indexer.GetClosestPocket());
        //telemetry.addData("Location XY Rot = ", drive.localizer.getPose().position.x);
        // telemetry.addData("distance = ", pocketSensors.GetDetectedPocketDistance());


        telemetry.update();
    }
}