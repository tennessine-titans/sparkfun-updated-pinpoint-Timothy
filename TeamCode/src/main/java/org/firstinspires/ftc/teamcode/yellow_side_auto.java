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

@Config
@Autonomous(name = "yellow_side_auto", group = "Autonomous")
public class yellow_side_auto extends Timothy {
    public int target=0;

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
                .turnTo(3*Math.PI/2)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(new Pose2d(-36,63,3*Math.PI/2))
                //Hang second specimen
                .strafeToLinearHeading(new Vector2d(4,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(new Pose2d(-2,31,3*Math.PI/2))
                // Get third specimen off the wall
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-36, 62.5),Math.PI/2)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(new Pose2d(-36,64,3*Math.PI/2))
                //hang third specimen
                .strafeToLinearHeading(new Vector2d(2,29),3*Math.PI/2)
                .build();
        Action TrajectoryAction17 = drive.actionBuilder(new Pose2d(-5,30,3*Math.PI/2))
                // Get fourth specimen off the wall
                .setTangent( Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-36, 62.5),Math.PI/2)
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
        Actions.runBlocking(shoulder.shoulderoutOftheWay());
        Actions.runBlocking(elbow.elbowIntake());


        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        lift.pidf_Lift_Controller(),
                        active_intake.active_IntakeOn()
                        //batteryVoltage.batteryMonitor(),
                        /*
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
                            claw.openClaw(),
                            new ParallelAction(
                                TrajectoryAction12,
                                new SequentialAction(
                                    extendo.extendoOut(),
                                    intake.intakedown(),
                                    intakewheel.wheelFoward()
                                )
                            )
                        )*/
                )
        );
    }
}

