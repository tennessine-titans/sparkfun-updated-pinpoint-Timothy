package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

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
        //myOTOS = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        //SparkFunOTOS.Pose2D pos = myOTOS.getPosition();
        Pose2d initialPose = new Pose2d(-9, 63, 3*Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Extendo extendo= new Extendo();
        Shoulder shoulder = new Shoulder();
        Elbow elbow = new Elbow();
        Wheel wheel = new Wheel();
        Claw claw = new Claw();
        Intake intake = new Intake();

        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                // go to submersible
                .strafeToLinearHeading(new Vector2d(-3,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction12 = drive.actionBuilder(new Pose2d(-3,29,3*Math.PI/2))
        //Action TrajectoryAction12 = drive.actionBuilder(drive.pose)
                // drvie to first sample
                .setTangent(5*Math.PI/8)
                .splineToConstantHeading(new Vector2d(-36, 40),Math.PI)
                .splineToConstantHeading(new Vector2d(-36, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-45, 48),Math.PI)
                // go behind second sample
                .splineToConstantHeading(new Vector2d(-51, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-57, 48),Math.PI)
                //go behind third sample
                .splineToConstantHeading(new Vector2d(-60, 14),Math.PI)
                .splineToConstantHeading(new Vector2d(-66, 14),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-66, 48),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-36, 63),Math.PI/2)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(new Pose2d(-36,63,3*Math.PI/2))
                //Hang second specimen
                .strafeToLinearHeading(new Vector2d(-2,31),3*Math.PI/2)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(new Pose2d(-2,31,3*Math.PI/2))
                // Get third specimen off the wall
                .setTangent(3 * Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-36, 63),Math.PI/2)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(drive.pose)
                //hang third specimen
                .strafeToLinearHeading(new Vector2d(-3,31),3*Math.PI/2)
                .build();
        Action TrajectoryAction17 = drive.actionBuilder(drive.pose)
                // Get fourth specimen off the wall
                .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                .build();
        Action TrajectoryAction18 = drive.actionBuilder(drive.pose)
                //hang fourth specimen
                .strafeToLinearHeading(new Vector2d(-3,31),3*Math.PI/2)
                .build();
        Action TrajectoryAction19 = drive.actionBuilder(drive.pose)
                // Get fifth specimen off the wall
                .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                .build();
        Action TrajectoryAction20 = drive.actionBuilder(drive.pose)
                //hang fifth specimen
                .strafeToLinearHeading(new Vector2d(-3,34),3*Math.PI/2)
                .build();
        Action WaitAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .build();

        Action WaitAction5 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();
        Action WaitAction51 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();
        Action WaitAction10 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        Action WaitAction11 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)

                .build();
        Action WaitAction300 = drive.actionBuilder(drive.pose)
                .waitSeconds(30)
                .build();
        Action WaitAction25 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction26 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction27 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction28 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction29 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction221 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction222 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction223 = drive.actionBuilder(drive.pose)
                .waitSeconds(.25)
                .build();
        Action WaitAction52 = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction53 = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();
        Action WaitAction54 = drive.actionBuilder(drive.pose)
                .waitSeconds(.50)
                .build();


        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.addData("Position X", drive.pose.position.x);
        telemetry.addData("Position Y", drive.pose.position.y);
        telemetry.update();
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(intake.intakeup());
        Actions.runBlocking(extendo.extednoIn());
        Actions.runBlocking(shoulder.shoulderoutOftheWay());
        Actions.runBlocking(elbow.elbowIntake());


        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        lift.pidf_Lift_Controller(),
                        new SequentialAction(
                                    //Drive to Submersible and hang first sample
                            new ParallelAction(
                                    TrajectoryAction11,
                                    new SequentialAction(
                                    lift.liftHangSample_PIDF(),
                                    shoulder.shoulderHangSpecimen(),
                                    elbow.elbowHang()
                                    )
                            ),
                            lift.liftExtraBump_PIDF(),
                                WaitAction51,
                            claw.openClaw(),
                                WaitAction25,
                                new ParallelAction(
                                        //Push all the samples to the human player area and set the arms to pick off the wall
                                        TrajectoryAction12,
                                        new SequentialAction(
                                                shoulder.shouldertransition(),
                                                claw.closeClaw(),
                                                WaitAction26,
                                                elbow.elbowWall(),
                                                WaitAction27,
                                                shoulder.shoulderWall(),
                                                WaitAction28,
                                                claw.openClaw(),
                                                lift.liftWall_PIDF()
                                        )

                                ),
                                //grab 2nd sample off the wall and drive to the submersible
                                claw.closeClaw(),
                                WaitAction52,
                                lift.liftHangSample_PIDF(),
                                WaitAction53,
                                new ParallelAction(
                                        TrajectoryAction14,
                                        new SequentialAction(
                                        shoulder.shoulderHangSpecimen(),
                                        elbow.elbowHang()
                                        )
                                ),
                                //Hang Sample 2
                                lift.liftExtraBump_PIDF(),
                                WaitAction54,
                                claw.openClaw(),
                                WaitAction29,
                                //Drive to wall to pick sample 3
                                new ParallelAction(
                                        TrajectoryAction15,
                                        new SequentialAction(
                                                shoulder.shouldertransition(),
                                                elbow.elbowWall(),
                                                shoulder.shoulderWall(),
                                                WaitAction221,
                                                lift.liftWall_PIDF()
                                        )
                                ),
                                //Grab 3rd sample from the wall
                                claw.closeClaw(),
                                WaitAction222
                               /*
                               lift.liftHangSample_PIDF(),
                                new ParallelAction(
                                        TrajectoryAction15,
                                        new SequentialAction(
                                                shoulder.shoulderHangSpecimen(),
                                                elbow.elbowHang()
                                        )

                                ),
                                lift.liftExtraBump_PIDF(),
                                claw.openClaw(),
                                WaitAction25,
                                new ParallelAction(
                                        TrajectoryAction16,
                                        new ParallelAction(
                                                shoulder.shouldertransition(),
                                                elbow.elbowWall(),
                                                shoulder.shoulderWall(),
                                                WaitAction25,
                                                lift.liftWall_PIDF()
                                        )
                                ),
                                claw.closeClaw(),
                                WaitAction25,
                                lift.liftHangSample_PIDF(),
                                new ParallelAction(
                                        TrajectoryAction17,
                                        new SequentialAction(
                                                shoulder.shoulderHangSpecimen(),
                                                elbow.elbowHang()
                                        )
                                ),
                                lift.liftExtraBump_PIDF(),
                                claw.openClaw(),
                                WaitAction25,
                                new ParallelAction(
                                        TrajectoryAction18,
                                        new SequentialAction(
                                                shoulder.shouldertransition(),
                                                elbow.elbowWall(),
                                                shoulder.shoulderWall(),
                                                WaitAction25,
                                                lift.liftWall_PIDF()
                                        )
                                ),
                                claw.closeClaw(),
                                WaitAction25,
                                lift.liftHangSample_PIDF(),
                                new ParallelAction(
                                        TrajectoryAction19,
                                        new SequentialAction(
                                                shoulder.shoulderHangSpecimen(),
                                                elbow.elbowHang()
                                        )
                                ),
                                lift.liftExtraBump_PIDF(),
                                claw.openClaw(),
                                WaitAction25,
                                TrajectoryAction20
                                */
                        )
                )
        );
    }
}

