package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robomossystem.MoMoreBotsDrivetrain;
import org.firstinspires.ftc.teamcode.robomossystem.bucketElevator;
import org.firstinspires.ftc.teamcode.robomossystem.intakeSubSystem;
import org.firstinspires.ftc.teamcode.robomossystem.specimenElevator;
import org.firstinspires.ftc.teamcode.utility.Constants;

@Autonomous(name="Score Bucket", group = "Into the Deep",preselectTeleOp = "Field Centered Teleop")
public class AutonBucket extends LinearOpMode
{
    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    boolean dummy;

    // get an instance of each subsystem class.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private bucketElevator bucketElevator = new bucketElevator(this);
    private specimenElevator specimenElevator = new specimenElevator(this);
    private intakeSubSystem intakeSubSystem = new intakeSubSystem(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        drivetrain.initialize(2);
        bucketElevator.init(true);
        specimenElevator.init(true);
        intakeSubSystem.init();

        int step = 1;

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions.
             */
            specimenElevator.periodic();
            bucketElevator.periodic();
            dummy =drivetrain.periodic(); //This is called as a variable for scheduling reasons
            intakeSubSystem.periodic();
            // We will be putting steps in here

        switch (step){
            case 1 :
                intakeSubSystem.armMidPosition();
                sleep(375);
                bucketElevator.highBucket();
               // drivetrain.gotoPosition(13,129.5,0,.2,1);
                //Move away from wall
                drivetrain.gotoPosition(16.5,110,0,.75,0);
                drivetrain.gotoPosition(5.5,130,-45,.75,.5);
                //Dump sample 0 into bucket
                bucketElevator.servoDump();
                //Wait until bucket is dumped.
                sleep(650);
                //Move bucket servo to receive positiob
                bucketElevator.servoRecieve();
                //Lower Bucket Elevator
                bucketElevator.toDown();
                step=step+1;
            case 2 :
                step=step+1;
                //drivetrain.gotoPosition(20,127,-45,.2,0);
               // drivetrain.gotoPosition(20,127,0,.2,0);
                //drivetrain.gotoPosition(26,128,0,.2,0);
                //Move to position near the 1st sample
                drivetrain.gotoPosition(23,128,-25,.75,.25);
                intakeSubSystem.armDownPosition();
                sleep(250);
                intakeSubSystem.intakeReverse();
                //When intake is running move arm slide forward to gobble the sample. We also drive forward.
                intakeSubSystem.intakeSlideForward();
                drivetrain.gotoPosition(37,119,-23,.75,0.3);
                intakeSubSystem.intakeStop();
                sleep(150);
                //intakeSubSystem.intakeSlideReverse();
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(250);
                //Dump sample 1 into bucket
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(22,121,0,.75,.15);

            case 3 :
                //Move arm to mid to clear the bucket
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                sleep(250);
                //Elevate bucket to high position
                bucketElevator.highBucket();
                //drivetrain.gotoPosition(15,129.5,0,.2,0);
                drivetrain.gotoPosition(5.5,130,-45,.75,.5);
                bucketElevator.servoDump();
                sleep(750);
                bucketElevator.servoRecieve();
                //After dump, drop bucket down
                bucketElevator.toDown();
                step=step+1;

            case 4:
                drivetrain.gotoPosition(17,129,0,.75,0);
                intakeSubSystem.intakeSlideOutAndArmDown();
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(33,129,0,.75,.45);
                intakeSubSystem.intakeStop();
                sleep(250);
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(250);
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(24,129,0,.75,.15);
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                sleep(250);
                bucketElevator.highBucket();
                //drivetrain.gotoPosition(15,129.5,0,.2,0);
                drivetrain.gotoPosition(5.5,130,-45,.75,.35);
                bucketElevator.servoDump();
                sleep(650);
                bucketElevator.servoRecieve();
                bucketElevator.toDown();
                //Get last sample
                drivetrain.gotoPosition(29,127,35,.75,0.1);
                intakeSubSystem.intakeSlideOutAndArmDown();
                intakeSubSystem.intakeReverse();
                sleep(650);
                drivetrain.gotoPosition(34.75,130.25,35,.75,.45);
                intakeSubSystem.intakeStop();
                sleep(250);
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(500);
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(24,129,0,.5,.15);
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                sleep(250);
                bucketElevator.highBucket();
                drivetrain.gotoPosition(6,130,-45,.5,.35);
                bucketElevator.servoDump();
                sleep(750);
                bucketElevator.servoRecieve();
                bucketElevator.LowBar();

                drivetrain.gotoPosition(69,115,90,.55,0);
                drivetrain.gotoPosition(69.5,98,90,.55,0);
                bucketElevator.servoDump();
                sleep(2000);
        }
            //update dashboard and telemetry if used
            if (Constants.Telemetry.showTelemetry) {
                telemetry.update();
            }
            if (Constants.Telemetry.showDashBoard) {
                packet.put("OTOS Heading", drivetrain.otosHead);
                packet.put("OTOS X", drivetrain.otosXPostion);
                packet.put("OTOS Y", drivetrain.otosYPostion);
                packet.put("Bot Heading", drivetrain.heading);
                packet.put("Left Front Power", drivetrain.LFpower);
                packet.put("Left Rear Power", drivetrain.LRpower);
                packet.put("Right Front Power", drivetrain.RFpower);
                packet.put("Right Rear Power", drivetrain.RRpower);
                packet.put("Specimen Elevator Position", specimenElevator.position);
                packet.put("Specimen Elevator Target", specimenElevator.target);
                packet.put("Specimen Elevator Power", specimenElevator.power);
                packet.put("Specimen Elevator at Target", specimenElevator.AtTarget);
                packet.put("Bucket Elevator Position", bucketElevator.position);
                packet.put("Bucket Elevator Target", bucketElevator.target);
                packet.put("Bucket Elevator at Target", bucketElevator.AtTarget);
                packet.put("Bucket Elevator Motor Power", bucketElevator.power);

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}