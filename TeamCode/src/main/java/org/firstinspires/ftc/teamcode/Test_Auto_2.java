package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.teamcode.teleops;



import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.InstantAction;
@Config
@Autonomous(name = "Test_Auto_2", group = "Autonomous")
public class Test_Auto_2 extends Timothy {
    public int target=0;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-9, 63, 3*Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Extendo extendo= new Extendo();
        Shoulder shoulder = new Shoulder();
        Elbow elbow = new Elbow();
        IntakeWheel intakewheel = new IntakeWheel();
        Claw claw = new Claw();
        Intake intake = new Intake();
        BatteryVoltage batteryVoltage= new BatteryVoltage(hardwareMap);

        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                // go to submersible
                .strafeToLinearHeading(new Vector2d(5,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction12 = drive.actionBuilder(new Pose2d(-3,29,3*Math.PI/2))
                 // drvie to first sample
                .setTangent(Math.PI/2)
                //Go away from submersable
                .splineToConstantHeading(new Vector2d(-34, 40),Math.PI)
                //.splineToConstantHeading(new Vector2d(-36, 40),Math.PI)
                //To get behind first sample
                .splineToConstantHeading(new Vector2d(-31, 14),Math.PI)
                .splineToConstantHeading(new Vector2d(-40, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-45, 48),Math.PI)
                // go behind second sample
                .splineToConstantHeading(new Vector2d(-53, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-57, 48),Math.PI)
                //go behind third sample
                .splineToConstantHeading(new Vector2d(-60, 14),Math.PI)
                .splineToConstantHeading(new Vector2d(-66, 14),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-66, 48),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-40, 48),Math.PI/2)
                //pick up off wall
                .splineToConstantHeading(new Vector2d(-38, 57),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-36, 64),Math.PI/2)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(new Pose2d(-36,63,3*Math.PI/2))
                //Hang second specimen
                .strafeToLinearHeading(new Vector2d(4,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(new Pose2d(-2,31,3*Math.PI/2))
                // Get third specimen off the wall
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-32, 50),2*Math.PI/3)
                .splineToConstantHeading(new Vector2d(-36, 64),Math.PI/2)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(new Pose2d(-36,64,3*Math.PI/2))
                //hang third specimen
                .strafeToLinearHeading(new Vector2d(2,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction17 = drive.actionBuilder(new Pose2d(-5,30,3*Math.PI/2))
                // Get fourth specimen off the wall
                .setTangent( Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-32, 50),2*Math.PI/3)
                .splineToConstantHeading(new Vector2d(-36, 64),Math.PI/2)
                .build();
        Action TrajectoryAction18 = drive.actionBuilder(new Pose2d(-36,64,3*Math.PI/2))
                //hang fourth specimen
                .strafeToLinearHeading(new Vector2d(0,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction19 = drive.actionBuilder(new Pose2d(-6,31,3*Math.PI/2))
                // Get fifth specimen off the wall
                .setTangent(3*Math.PI / 8)
                .splineToConstantHeading(new Vector2d(-36, 62.5),Math.PI/2)
                .build();
        Action TrajectoryAction20 = drive.actionBuilder(new Pose2d(-36,31,3*Math.PI/2))
                //hang fifth specimen
                .strafeToLinearHeading(new Vector2d(-2,29),3*Math.PI/2)
                .build();
        Action WaitAction10A = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10B = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10C = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10D = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10E = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10F = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10G = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10H = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10I = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10J = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();
        Action WaitAction10K = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();

        Action WaitAction5A = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5B = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5C = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5D = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5E = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5F = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction5G = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();

        Action WaitAction1A = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        Action WaitAction1B = drive.actionBuilder(drive.pose)
                .waitSeconds(1)

                .build();
        Action WaitAction300 = drive.actionBuilder(drive.pose)
                .waitSeconds(30)
                .build();
        Action WaitAction25A = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25B = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25C = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25D = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25E = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25F = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25G = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25H = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25I = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25J = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25K = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25L = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25M = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25N = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25O = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25P = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25Q = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction25R = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();


        Action WaitAction75A= drive.actionBuilder(drive.pose)
                .waitSeconds(.75)
                .build();
        Action WaitAction75B= drive.actionBuilder(drive.pose)
                .waitSeconds(.75)
                .build();
        Action WaitAction75C= drive.actionBuilder(drive.pose)
                .waitSeconds(.75)
                .build();

        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.addData("Position X", drive.pose.position.x);
        telemetry.addData("Position Y", drive.pose.position.y);
        telemetry.update();
        Actions.runBlocking(claw.closeClaw());
        //Actions.runBlocking(intake.intakeup());
        Actions.runBlocking(extendo.extednoIn());
        Actions.runBlocking(elbow.elbowredAutoInt());
        Actions.runBlocking(shoulder.shoulderRedAutoInt());


        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        lift.pidf_Lift_Controller(),
                        //batteryVoltage.batteryMonitor(),
                        new SequentialAction(
                            //Drive to Submersible and hang first sample
                            new ParallelAction(
                                    TrajectoryAction11,
                                    new SequentialAction(
                                        lift.liftHangSample_PIDF(),
                                        shoulder.shoulderHangSpecimen(),
                                        WaitAction10A,// 0.1
                                        elbow.elbowHang()
                                    )
                            ),
                            lift.liftExtraBump_PIDF(),
                            //WaitAction5A,  //Might be able to remove  0.5
                            claw.openClaw(),  // changed from claw.openClaw();
                            WaitAction25A,    // might be able to remove  0.25
                            new ParallelAction(
                                    //Push all the samples to the human player area and set the arms to pick off the wall
                                 TrajectoryAction12,
                                 new SequentialAction(
                                      WaitAction5B,  //0.5
                                      claw.closeClaw(),
                                      shoulder.shouldertransition(),
                                      WaitAction25B,  // 0.25
                                      elbow.elbowWall(),
                                      WaitAction25C,  // 0.25
                                      shoulder.shoulderWall(),
                                      WaitAction25D,  //0.25
                                      lift.liftWall_PIDF(),
                                      claw.clawWall()
                                 )
                            ),
                                //grab 2nd sample off the wall and drive to the submersible
                            claw.closeClaw(),
                            WaitAction5C,  //0.5
                            lift.liftHangSample_PIDF(),
                            //WaitAction5D, // might be able to be removed  0.5
                            new ParallelAction(
                                TrajectoryAction14,
                                new SequentialAction(
                                    shoulder.shoulderHangSpecimen(),
                                    WaitAction10B,
                                    elbow.elbowHang()
                                    )
                            ),
                                //Hang Sample 2
                            lift.liftExtraBump_PIDF(),
                            //WaitAction75A,  // might be able to be removed  0.75
                            claw.openClaw(), //changed from claw.openClaw(),
                                //Drive to wall to pick sample 3
                                WaitAction25R,
                            new ParallelAction(
                                TrajectoryAction15,
                                new SequentialAction(
                                    WaitAction25E,  // may be able to move to line 287  0.25
                                    shoulder.shouldertransition(),
                                    claw.closeClaw(),
                                    WaitAction10C,
                                    elbow.elbowWall(),
                                    WaitAction10D,
                                    shoulder.shoulderWall(),
                                    WaitAction25G,
                                    claw.clawWall(),
                                    lift.liftWall_PIDF()
                                )
                            ),
                                //Grab 3rd sample from the wall
                            claw.closeClaw(),
                            WaitAction25H, //0.25
                            lift.liftHangSample_PIDF(),
                            //WaitAction25I, // may be able to be removed  0.25
                            new ParallelAction(
                                TrajectoryAction16,
                                new SequentialAction(
                                    shoulder.shoulderHangSpecimen(),
                                    WaitAction10E,
                                    elbow.elbowHang()
                                )
                            ),
                            //Hang 3rd
                            lift.liftExtraBump_PIDF(),
                            //WaitAction75B, // may be able to be removed 0.75
                            claw.openClaw(),//was claw.openClaw(),
                            new ParallelAction(
                                TrajectoryAction17,
                                new SequentialAction(
                                    WaitAction5E,  // 0.5  may be able to move to line 319
                                    shoulder.shouldertransition(),
                                    claw.closeClaw(),
                                    WaitAction10F,
                                    elbow.elbowWall(),
                                    WaitAction25J,
                                    shoulder.shoulderWall(),
                                    WaitAction25O,
                                    claw.clawWall(),
                                    WaitAction10K,  //.25
                                    lift.liftWall_PIDF()
                                )
                            ),
                                //grab 4th
                            claw.closeClaw(),
                            WaitAction5F,  //0.5
                            lift.liftHangSample_PIDF(),
                            //WaitAction5G,  // may be able to be removed 0.5
                            new ParallelAction(
                                TrajectoryAction18,
                                new SequentialAction(
                                   shoulder.shoulderHangSpecimen(),
                                   WaitAction10G,
                                   elbow.elbowHang()
                                )
                            ),
                            lift.liftExtraBump_PIDF(),
                            //WaitAction75C,  // may be able to be removed  0.75
                            claw.openClaw(),  // was claw.openClaw(),
                            WaitAction25P,
                            new ParallelAction(
                            TrajectoryAction19,
                                new SequentialAction(
                                WaitAction5D,
                                elbow.elbowIntake(),
                                WaitAction25Q,
                                shoulder.shoulderintake(),
                                lift.liftWall_PIDF()
                                )
                            )

                        )
                )
        );
    }
}

