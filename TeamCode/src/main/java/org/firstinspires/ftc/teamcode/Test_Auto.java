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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Test_Auto", group = "Autonomous")
public class Test_Auto extends LinearOpMode {
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


        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.9);
                    lift2.setPower(-0.9);


                    initialized = true;
                }

                //double leftpos = lift1.getCurrentPosition();
                //double rightpos = lift2.getCurrentPosition();
                //packet.put("liftPos", leftpos);
                //packet.put("liftPos", rightpos);
                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);


                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> -1400.0) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftHalfUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.9);
                    lift2.setPower(-0.9);


                    initialized = true;
                }

                //double leftpos = lift1.getCurrentPosition();
                //double rightpos = lift2.getCurrentPosition();
                //packet.put("liftPos", leftpos);
                //packet.put("liftPos", rightpos);
                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);


                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> -700.0) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftHalfUp() {
            return new LiftHalfUp();
        }


        public class LiftHangSample implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.3);
                    lift2.setPower(0.3);
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);


                // ToDo determine how many ticks represents lift up (left + right)
                if (pos < -950 ) {
                    telemetry.addData("Position ",pos );
                    telemetry.update();
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftHangSample(){
            return new LiftHangSample();
        }


        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.8);
                    lift2.setPower(0.8);
                    initialized = true;
                }

                double leftpos = lift1.getCurrentPosition();
                double rightpos = lift2.getCurrentPosition();
                packet.put("liftPos", leftpos);
                packet.put("liftPos", rightpos);
                if (leftpos+rightpos < -100.0) {
                    telemetry.addData("Position ",leftpos + rightpos );
                    telemetry.update();
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ArmBucket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.5);
                    initialized = true;
                }

                double armpos = arm.getCurrentPosition();
                packet.put("armPos", armpos);
                //ToDo Determine arm position to place sample on high bar
                if (armpos < -10) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armBucket() {
            return new ArmBucket();
        }
        public class ArmBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-0.5);
                    initialized = true;
                }

                double armpos = arm.getCurrentPosition();
                packet.put("armPos", armpos);
                if (armpos > -320) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armBasket() {
            return new ArmBasket();
        }

    }
    public class Human {
        private Servo human;

        public Human(HardwareMap hardwareMap) {
            human = hardwareMap.get(Servo.class, "human");

        }

        public class CloseHuman implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                human.setPosition(0.1);
                return false;
            }
        }
        public Action closeHuman() {
            return new CloseHuman();
        }

        public class OpenHuman implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                human.setPosition(.5);
                return false;
            }
        }
        public Action openHuman() {
            return new OpenHuman();
        }
    }

    public class Hand {
        private Servo hand;

        public Hand(HardwareMap hardwareMap) {
            hand = hardwareMap.get(Servo.class, "hand");

        }


        public class CloseHand implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hand.setPosition(0);
                return false;
            }
        }
        public Action closeHand() {
            return new CloseHand();
        }

        public class OpenHand implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hand.setPosition(0.5);
                return false;
            }
        }
        public Action openHand() {
            return new OpenHand();
        }
    }
    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");

        }


        public class WristBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.7);
                return false;
            }
        }
        public Action wristBucket() {
            return new WristBucket();
        }

        public class WristBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.25);
                return false;
            }
        }
        public Action wristBasket() {
            return new WristBasket();
        }
    }
    public class Intake1 {
        private Servo intake1;

        public Intake1(HardwareMap hardwareMap) {
            intake1 = hardwareMap.get(Servo.class, "intake1");

        }


        public class Intake1Tilted implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake1.setPosition(0.3);
                return false;
            }
        }
        public Action Intake1Tilted() {
            return new Intake1Tilted();
        }

        public class Intake1Straight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake1.setPosition(1);
                return false;
            }
        }
        public Action Intake1Straight() {
            return new Intake1Straight();
        }

        public class Intake1Bucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake1.setPosition(.8);
                return false;
            }
        }
        public Action Intake1Bucket() {
            return new Intake1Bucket();
        }
    }
    public class Intake2 {
        private Servo intake2;

        public Intake2(HardwareMap hardwareMap) {
            intake2 = hardwareMap.get(Servo.class, "intake2");

        }


        public class Intake2Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake2.setPosition(0.0);
                return false;
            }
        }
        public Action Intake2Up() {
            return new Intake2Up();
        }

        public class Intake2Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake2.setPosition(0.53);
                return false;
            }
        }
        public Action Intake2Down() {
            return new Intake2Down();
        }
    }
    public class Intakewheel {
        private CRServo intakewheel;

        public Intakewheel(HardwareMap hardwareMap) {
            intakewheel = hardwareMap.get(CRServo.class, "intakewheel");

        }


        public class Intakewheelin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakewheel.setPower(-1);
                return false;
            }
        }
        public Action Intakewheelin() {
            return new Intakewheelin();
        }

        public class Intakewheelout implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakewheel.setPower(1);
                return false;
            }
        }
        public Action Intakewheelout() {
            return new Intakewheelout();
        }
        public class Intakewheelstop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakewheel.setPower(0);
                return false;
            }
        }
        public Action Intakewheelstop() {
            return new Intakewheelstop();
        }
    }
    public class Intakearm {
        private DcMotorEx intakearm;

        public Intakearm(HardwareMap hardwareMap) {
            intakearm = hardwareMap.get(DcMotorEx.class, "intakearm");
            intakearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakearm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakearmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakearm.setPower(0.9);
                    initialized = true;
                }

                double intakearmpos = intakearm.getCurrentPosition();
                packet.put("intakearmPos", intakearmpos);
                //ToDo Determine arm position to place sample on high bar
                if (intakearmpos < -10) {
                    return true;
                } else {
                    intakearm.setPower(0);
                    return false;
                }
            }
        }
        public Action intakearmUp() {
            return new IntakearmUp();
        }
        public class IntakearmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakearm.setPower(-0.2);
                    initialized = true;
                }

                double intakearmpos = intakearm.getCurrentPosition();
                packet.put("intakearmPos", intakearmpos);
                if (intakearmpos > -200) {
                    return true;
                } else {
                    intakearm.setPower(0);
                    return false;
                }
            }
        }
        public Action intakearmDown() {
            return new IntakearmDown();
        }
        public class IntakearmHold implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakearm.setPower(0.1);
                return false;




            }
        }
        public Action intakearmHold() {
            return new IntakearmHold();
        }

    }
    @Override
    public void runOpMode() {
        //myOTOS = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        //SparkFunOTOS.Pose2D pos = myOTOS.getPosition();
        Pose2d initialPose = new Pose2d(-9, 63, 3*Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                //Todo forward to hang preloaded specimen
                .strafeToLinearHeading(new Vector2d(-7,30),3*Math.PI/2)
                .build();
        Action TrajectoryAction12 = drive.actionBuilder(drive.pose)
                /*.setTangent(3*Math.PI/4)
                .splineToConstantHeading(new Vector2d(-15, 45),3*Math.PI/2)
                .splineToConstantHeading(new Vector2d(-37,55),3*Math.PI/2)
                .splineToConstantHeading(new Vector2d(-37,16),3*Math.PI/2)
                .splineToConstantHeading(new Vector2d(-48,15),3*Math.PI/2)
                .splineToConstantHeading(new Vector2d(-48,52),3*Math.PI/2)
                //TODO Push second sample to the wall
                .splineToLinearHeading(new Pose2d(-46,20,Math.PI/2),Math.PI)
                .splineToConstantHeading(new Vector2d(-57,20),5*Math.PI/8)
                .splineToLinearHeading(new Pose2d(-56,61, Math.PI/2),Math.PI/2)
                .splineToLinearHeading(new Pose2d(-49,64,Math.PI/2),Math.PI/2)
                .build();
        */
                .setTangent(3*Math.PI/4)
                //TODO Crab over away from submersible
                .splineToConstantHeading(new Vector2d(-15, 45),3*Math.PI/4)
                .splineToConstantHeading(new Vector2d(-36,55),3*Math.PI/2)
                //TODO Get beside 1 sample
                .splineToConstantHeading(new Vector2d(-33,22),3*Math.PI/2)
                //TODO Get behind 1 sample
                .splineToConstantHeading(new Vector2d(-50,22),Math.PI/2)
                //TODO push 1 sample into human player area
                .splineToConstantHeading(new Vector2d(-50,56),3*Math.PI/2)

                //TODO Back up to get 2 sample
                .splineToSplineHeading(new Pose2d(-46,20,Math.PI/2),Math.PI)
                //TODO Get behind 2 sample
                .splineToSplineHeading(new Pose2d(-62,20,5*Math.PI/8),Math.PI/2)
                // TODO Push second sample to the wall
                .splineToSplineHeading(new Pose2d(-62,54, 5*Math.PI/8),Math.PI/2)
                // TODO line up to grab second specimen
                .splineToLinearHeading(new Pose2d(-49,65,Math.PI/2),Math.PI/2)
                .build();
        Action TrajectoryAction3 = drive.actionBuilder(drive.pose)
                //TODO Hang second specimen
                .strafeToLinearHeading(new Vector2d(0, 30),  3*Math.PI/2)
                .build();
        Action TrajectoryAction14 = drive.actionBuilder(drive.pose)
                //TODO Pick up 3rd Specimen
                .strafeToLinearHeading(new Vector2d(-49,65),Math.PI/2)
                .build();
        Action TrajectoryAction15 = drive.actionBuilder(drive.pose)
                //TODO Hang third specimen
                .strafeToLinearHeading(new Vector2d(-2, 29.5),  3*Math.PI/2)
                .build();
        Action TrajectoryAction16 = drive.actionBuilder(drive.pose)
                //TODO go park
                .strafeToLinearHeading(new Vector2d(-49,65),Math.PI/2)
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






        // actions that need to happen on init; for instance, a claw tightening.

        //Actions.runBlocking((intakearm.intakearmHold()));


        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.addData("Position X", drive.pose.position.x);
        telemetry.addData("Position Y", drive.pose.position.y);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(

                new SequentialAction(
                        new ParallelAction(


                                TrajectoryAction11


                        ),

                        WaitAction1,
                        new ParallelAction(
                                TrajectoryAction12

                        ),
                        //TODO Grab Second Sample

                        WaitAction10,

                        new ParallelAction(

                                //TODO Drive to Hang Sample
                                TrajectoryAction3
                        ),

                        //TODO Hang Second Sample

                        //TODO Drive to Third Sample
                        new ParallelAction(

                                TrajectoryAction14
                        ),
                        //TODO  Grab Third Sample

                        WaitAction11,

                        new ParallelAction(

                                TrajectoryAction15
                        ),


                        //Todo Hang Third Sample

                        WaitAction5,
                        //TODO park
                        new ParallelAction(

                                TrajectoryAction16
                        )

                )
        );
    }
}