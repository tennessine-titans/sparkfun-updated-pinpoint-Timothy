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
public class Test_Auto_2 extends LinearOpMode {
    public int target=0;
    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;


        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //  ToDO Set Motor Direction if Necessary
            lift1.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public class PIDF_Lift_Controller implements Action {
            private boolean initialized = false;
            public double p = 0.01, i = 0, d = 0.00001, f = -0.05;
            //public int target = 0;

            private PIDController controller;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    controller = new PIDController(p, i, d);
                    controller.setPID(p, i, d);
                    int lift1_pos = lift1.getCurrentPosition();
                    int lift2_pos = lift2.getCurrentPosition();
                    int lift_avg = (lift1_pos + lift2_pos) / 2;
                    double pid = controller.calculate(lift_avg, target);
                    double power = pid + f;
                    lift1.setPower(power);
                    lift2.setPower(power);
                    telemetry.addData("lift_avg", lift_avg);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                return true;
            }
        }

        public Action pidf_Lift_Controller() {
            return new PIDF_Lift_Controller();
        }
    }
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
                .splineToConstantHeading(new Vector2d(-40, 60),Math.PI/2)
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
                                    TrajectoryAction11,
                                    new InstantAction(()->target=500)
                            )
                    )
                )
        );
    }
}