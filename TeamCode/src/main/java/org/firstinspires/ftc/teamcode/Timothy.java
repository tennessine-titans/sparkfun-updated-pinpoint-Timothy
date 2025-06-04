package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public abstract class Timothy extends LinearOpMode {
    // Variables and actions used to set and run Timothys servo postions and motor positions.
    //Set default values;
    protected double intakeDown = .58;// to pick up sample
    protected double intakeUp = .26; // postiton when extendo is retracted
    protected double intakeWheelforward = 1;
    protected double intakeWheelbackward = -1;
    protected double intakeWheeloff = 0;
    protected double leftExtendoOut = 0.59;
    protected double rightExtendoOut = 0.59;
    // Define as servos
    protected double leftExtendoIn = 0.09;
    protected double rightExtendoIn= .09;
    protected double rightShoulderintake = 0.5;
    protected double rightElbowintake = 0.25;
    protected double leftShoulderintake = 0.5;
    protected double leftElbowintake = 0.25;
    protected double rightShoulderbasket = 0.66;
    protected double leftShoulderbasket = 0.66;
    protected double rightElbowbasket = .94;
    protected double leftElbowbasket = .94;
    protected double clawClosed = .728;
    protected double clawOpen = .328;
    protected double leftShoulderspecimenTransition = .6;
    protected double rightShoulderspecimenTransition = .6;
    protected double leftElbowWall = .005;
    protected double rightElbowWall = .005;
    protected double leftShoulderWall = .41;
    protected double rightShoulderWall = .41;
    protected double leftExtendohalf = .36;
    protected double rightExtendohalf = .36;
    protected double rightShoulderhangSpecimen = .75;
    protected double leftShoulderhangSpecimen = .75;
    protected double leftElbowhangSpecimen = .25;
    protected double rightElbowhangSpecimen = .25;
    protected double leftElbowextraBump = .183;
    protected double rightElbowextraBump =.183;
    protected double rightShoulderoutOftheWay =.55;
    protected double leftShoulderoutOftheWay =.55;
    protected double p = 0.01;
    protected double i = 0.013;
    protected double d = 0.0004;
    protected double f = 0.06;


    public int target = 0;
    //Define servos and motors
    public Servo Lextendo;
    public Servo Rextendo;
    public Servo intakePosition;
    public CRServo intakeWheel;
    public Servo leftShoulder;
    public Servo rightShoulder;
    public Servo leftElbow;
    public Servo rightElbow;
    public Servo claw;
    public DcMotor lift1;
    public DcMotor lift2;
    //public ColorSensor intakeSensor;
    //public ColorSensor clawSensor;
    public void intLextendo(){
        Lextendo = hardwareMap.get(Servo.class, "Lextendo");
    }
    public void intRextendo(){
        Rextendo = hardwareMap.get(Servo.class, "Rextendo");
    }
    public void intintakePosition(){
        intakePosition = hardwareMap.get(Servo.class, "intakePosition");
    }
    public void intintakeWheel(){
        intakeWheel = hardwareMap.get(CRServo.class, "intakeWheel");
    }
    public void intrightShoulder(){
        rightShoulder = hardwareMap.get(Servo.class, "rightShoulder");
    }
    public void intleftShoulder(){
        leftShoulder = hardwareMap.get(Servo.class, "leftShoulder");
    }
    public void intrightElbow(){
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");
    }
    public void intleftElbow(){
        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
    }
    public void intclaw(){
        claw = hardwareMap.get(Servo.class, "claw");
    }
    public void intlift1(){
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void intlift2(){
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    //public void intintakeSensor() {
        //intakeSensor = hardwareMap.get(ColorSensor.class, "intakeSensor");

        //public void intclawSensor() {
           // clawSensor = hardwareMap.get(ColorSensor.class, "clawSensor");


    //PIDF Controller Class for the lifts only need to change target value for controller to drive lift motors.
   /*
    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;


        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //  ToDO Set Motor Direction if Necessary
            lift1.setDirection(DcMotorSimple.Direction.REVERSE);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
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
        public class LiftUp_PIDF implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=1400;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target-50) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
                    return true;
                } else {

                    return false;
                }
            }
        }
        public Action liftUp_PIDF() {
            return new LiftUp_PIDF();
        }


        public class LiftDown_PIDF implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=10;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target+50) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
                    return true;
                } else {

                    return false;
                }
            }
        }
        public Action liftDown_PIDF() {
            return new LiftDown_PIDF();
        }


    }



/*
    public void main() {
        extendoOut();
    }

    public class Lextendo {
        private Servo lExtendo;

        public Lextendo(HardwareMap hardwareMap) {
            lExtendo = hardwareMap.get(Servo.class, "Lextendo");
            Lextendo.setPosition(leftExtendoOut);
        }
    }
    public class Rextendo {
        private Servo rExtendo;

        public Rextendo(HardwareMap hardwareMap) {
            rExtendo = hardwareMap.get(Servo.class, "Rextendo");

        }
    }
    public void extendoOut() {

        Rextendo.setPosition(rightExtendoOut);

 */

    }










