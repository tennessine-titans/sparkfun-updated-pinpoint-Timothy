package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.teamcode.teleops;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import javax.annotation.Nullable;

@Config
@Autonomous(name = "yellow_side_auto", group = "Autonomous")
public class yellow_side_auto extends Timothy {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(38, 63, Math.PI);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Extendo extendo= new Extendo();
        Shoulder shoulder = new Shoulder();
        Elbow elbow = new Elbow();
        IntakeWheel intakewheel = new IntakeWheel();
        Claw claw = new Claw();
        Intake intake = new Intake();
        BatteryVoltage batteryVoltage= new BatteryVoltage(hardwareMap);
        Active_Intake active_intake= new Active_Intake();

        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                // go to basket
                .strafeToLinearHeading(new Vector2d(55,58),5*Math.PI/4)
                .build();

        Action TrajectoryAction12 = drive.actionBuilder(new Pose2d(55,58,5*Math.PI/4))
                // pick second first sample
                .strafeToLinearHeading(new Vector2d(58,51),11*Math.PI/8)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(new Pose2d(58,51,11*Math.PI/8))
                //put second sample in basket
                .strafeToLinearHeading(new Vector2d(55,58),5*Math.PI/4)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(new Pose2d(55,58,5*Math.PI/4))
                // pick up third sample
                .strafeToLinearHeading(new Vector2d(43,48),5*Math.PI/3)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(new Pose2d(43,50,5*Math.PI/3))
                //place third sample
                .strafeToLinearHeading(new Vector2d(55,57),5*Math.PI/4)
                .build();
        Action TrajectoryAction17 = drive.actionBuilder(new Pose2d(55,57,5*Math.PI/4))
                // Get fourth sample
                .strafeToLinearHeading(new Vector2d(50.5, 43),7*Math.PI/4)
                .build();
        Action TrajectoryAction18 = drive.actionBuilder(new Pose2d(50.5,45,7*Math.PI/4))
                //place fourth sample
                .strafeToLinearHeading(new Vector2d(55,57),5*Math.PI/4)
                .build();
        Action TrajectoryAction19 = drive.actionBuilder(new Pose2d(55,57,5*Math.PI/4))
                //park
                .splineToLinearHeading(new Pose2d(45,12,Math.PI),Math.PI)
                .splineToLinearHeading(new Pose2d(20,12,Math.PI),Math.PI)
                .build();
        Action TestTraj = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(30,30),3*Math.PI/2)
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
        Actions.runBlocking(intake.intakeup());
        Actions.runBlocking(extendo.extednoIn());
        Actions.runBlocking(shoulder.shoulderyellowAutoInt());
        Actions.runBlocking(elbow.elbowyellowAutoInt());


        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        lift.pidf_Lift_Controller(),
                        //batteryVoltage.batteryMonitor(),
                        new SequentialAction(
                            //Drive to Submersible and hang first sample
                            new ParallelAction(
                                TrajectoryAction11,
                                lift.liftUp_PIDF(),
                                new SequentialAction(
                                    shoulder.shoulderbasket(),
                                    elbow.elbowBasket()
                                )
                            ),
                        WaitAction25A,
                        claw.openClaw(),
                        WaitAction25L,
                        TrajectoryAction12,
                        active_intake.active_IntakeOn(),
                        new ParallelAction(
                                TrajectoryAction14,
                                new SequentialAction(
                                    elbow.elbowIntake(),
                                    WaitAction25B,
                                    shoulder.shoulderintake(),
                                    lift.liftDown_PIDF(),
                                    claw.closeClaw(),
                                    WaitAction75A,
                                    lift.liftUp_PIDF(),
                                    shoulder.shoulderbasket(),
                                    WaitAction25C,
                                    elbow.elbowBasket()
                                )
                        ),
                        WaitAction25D,
                        claw.openClaw(),
                        WaitAction25E,
                        TrajectoryAction15,
                        active_intake.active_IntakeOn(),
                        WaitAction5D,
                        new ParallelAction(
                            TrajectoryAction16,
                            new SequentialAction(
                                elbow.elbowIntake(),
                                WaitAction25F,
                                shoulder.shoulderintake(),
                                lift.liftDown_PIDF(),
                                claw.closeClaw(),
                                WaitAction75B,
                                lift.liftUp_PIDF(),
                                shoulder.shoulderbasket(),
                                WaitAction25G,
                                elbow.elbowBasket()
                            )
                        ),
                        WaitAction25H,
                        claw.openClaw(),
                        WaitAction25I,
                        TrajectoryAction17,
                        active_intake.active_IntakeOn(),
                        new ParallelAction(
                            TrajectoryAction18,
                            new SequentialAction(
                                elbow.elbowIntake(),
                                WaitAction25J,
                                shoulder.shoulderintake(),
                                lift.liftDown_PIDF(),
                                claw.closeClaw(),
                                WaitAction75C,
                                lift.liftUp_PIDF(),
                                shoulder.shoulderbasket(),
                                WaitAction25K,
                                elbow.elbowBasket()
                            )
                        ),
                        WaitAction25M,
                        claw.openClaw(),
                        WaitAction25O,
                        new ParallelAction(
                        TrajectoryAction19,
                                WaitAction25N,
                                lift.liftDown_PIDF(),
                                claw.closeClaw(),
                                shoulder.shoulderpark(),
                                elbow.elbowpark()
                        ),
                         shoulder.shoulderpark2()
                )
                )
        );
    }
}

