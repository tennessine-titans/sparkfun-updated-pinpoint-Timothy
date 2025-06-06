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

        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                // go to submersible
                .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                .build();
        Action TrajectoryAction12 = drive.actionBuilder(drive.pose)
                // drvie to first sample
                .setTangent(5*Math.PI/8)
                .splineToConstantHeading(new Vector2d(-30, 40),Math.PI)
                .splineToConstantHeading(new Vector2d(-38, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-45, 48),Math.PI)
                // go behind second sample
                .splineToConstantHeading(new Vector2d(-50, 14),Math.PI)
                //push sample to wall
                .splineToConstantHeading(new Vector2d(-55, 48),Math.PI)
                //go behind third sample
                .splineToConstantHeading(new Vector2d(-56, 14),Math.PI)
                .splineToConstantHeading(new Vector2d(-62, 14),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-55, 48),0)
                .build();
        Action TrajectoryAction13 = drive.actionBuilder(drive.pose)
                //Pick up second sample off the wall
                .splineToConstantHeading(new Vector2d(-40, 60),Math.PI/2)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(drive.pose)
                //Hang second specimen
                .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(drive.pose)
                // Get third specimen off the wall
                .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(drive.pose)
                //hang third specimen
                .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                .build();
        Action TrajectoryAction17 = drive.actionBuilder(drive.pose)
                // Get fourth specimen off the wall
                .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                .build();
        Action TrajectoryAction18 = drive.actionBuilder(drive.pose)
                //hang fourth specimen
                .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                .build();
        Action TrajectoryAction19 = drive.actionBuilder(drive.pose)
                // Get fifth specimen off the wall
                .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                .build();
        Action TrajectoryAction20 = drive.actionBuilder(drive.pose)
                //hang fifth specimen
                .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                .build();


        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.addData("Position X", drive.pose.position.x);
        telemetry.addData("Position Y", drive.pose.position.y);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        lift.pidf_Lift_Controller(),
                        new SequentialAction(
                            new ParallelAction(
                                    lift.liftHangSample_PIDF(),
                                    shoulderHangSpecimen(),
                                    elbowHang(),
                                    TrajectoryAction11
                            ),
                            openClaw()
                        )
                )
        );
    }
}

