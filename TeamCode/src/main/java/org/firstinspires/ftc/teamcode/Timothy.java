package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public abstract class Timothy extends LinearOpMode {
    // Variables and actions used to set and run Timothys servo positions and motor positions.
    //Set default values;
    //protected double intakeDown = .6;// to pick up sample when servo bloxk was on top of extendo
    protected double intakeDown = .49;// to pick up sample
    protected double intakeUp = .28; // postiton when extendo is retracted
    protected double intakeWheelforward = 1.0;
    protected double intakeWheelbackward = -1.0;
    protected double intakeWheeloff = 0.0;
    protected double leftExtendoOut = 0.59;
    protected double rightExtendoOut = 0.59;
    // Define as servos
    protected double leftExtendoIn = 0.11;
    protected double rightExtendoIn= 0.11;
    protected double rightShoulderintake = 0.46;
    protected double rightElbowintake = 0.31;
    protected double leftShoulderintake = 0.46;
    protected double leftElbowintake = 0.31;
    protected double rightShoulderbasket = 0.7;
    protected double leftShoulderbasket = 0.7;
    protected double rightElbowbasket = .89;
    protected double leftElbowbasket = .89;
    protected double clawClosed = .528;
    protected double clawRelease = .438;
    protected double clawOpen = .38;
    protected double leftShoulderspecimenTransition = .5;
    protected double rightShoulderspecimenTransition = .5;
    protected float leftElbowWall = 0;
    protected float rightElbowWall = 0;
    protected double leftShoulderWall = .38;
    protected double rightShoulderWall = .38;
    protected double leftExtendohalf = .36;
    protected double rightExtendohalf = .36;
    protected double rightShoulderhangSpecimen = .69;
    protected double leftShoulderhangSpecimen = .69;
    protected double leftElbowhangSpecimen = .35;
    protected double rightElbowhangSpecimen = .35;
    protected double leftElbowextraBump = .183;
    protected double rightElbowextraBump =.183;
    protected double rightShoulderoutOftheWay =.55;
    protected double leftShoulderoutOftheWay =.55;
    protected double leftShoulderpark = .63;
    protected double rightShoulderpark = .63;
    protected double leftElbowpark = .5;
    protected double rightElbowpark = .5;
    protected double clawWall = 0.308;
    protected double p = 0.01;
    protected double i = 0;
    protected double d = 0;
    protected double f = 0.001;
    public int target = 0;
    public int liftHangspecimen = 350;
    public int liftExtrabump = 950;
    public int liftDown =50;
    String intakecolorDetected;
    public int intakecolorDetectedvalue;
    String clawcolorDetected;
    public int clawcolorDetectedvalue;
    public float[] hsv = new float[3];
    public int red;
    public int blue;
    public int green=0;
    public float hue;
    public float saturation;
    public float value;
    public int machineState = 0;
    public double startTime;
    public int step = 0;
    public boolean scan = true;
    public boolean intakeDirectionForward =false;
    public boolean status =true;
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
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public ColorSensor intake1;

    public ColorSensor clawSensor;
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
        //lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void intlift2(){
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void definedrivemotors() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }
/*
    public void intintake1() {
        intake1 = hardwareMap.get(NormalizedColorSensor.class, "intake1");

    }
    public void intclawSensor() {
        clawSensor = hardwareMap.get(NormalizedColorSensor.class, "clawSensor");
    }

    public abstract class Colorsensor extends LinearOpMode{
        NormalizedColorSensor intake1;
        int scaleFactor = 1000;
        public Colorsensor(NormalizedColorSensor intake1){this.intake1= intake1;}
        public float[] ColorsensorCheck(){
            int red = Math.round(intake1.getNormalizedColors().red*255*scaleFactor);
            int green = Math.round(intake1.getNormalizedColors().green*255*scaleFactor);
            int blue = Math.round(intake1.getNormalizedColors().blue*255*scaleFactor);
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);
            return hsv;
        }
    }
    */

    //PIDF Controller Class for the lifts only need to change target value for controller to drive lift motors.
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
            public double p = 0.01, i = 0.0001, d = 0.0003, f = 0.02;
            //public int target = 0;

            private PIDController controller;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    controller = new PIDController(p, i, d);
                    controller.setPID(p, i, d);
                    int lift1_pos = lift1.getCurrentPosition();
                    //int lift2_pos = lift2.getCurrentPosition();
                    //int lift_avg = (lift1_pos + lift2_pos) / 2;
                    double pid = controller.calculate(lift1_pos, target);
                    double power = pid + f;
                    lift1.setPower(power);
                    lift2.setPower(power);
                    //telemetry.addData("lift_avg", lift_avg);
                    //telemetry.addData("target", target);
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
                    target=1490;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);
                telemetry.addData("Position ",pos);
                telemetry.update();
                // ToDo determine how many ticks represents lift up (left + right)
                if (pos< (target-10)) {

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
                    target=liftDown;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);
                telemetry.addData("Position ",pos);
                telemetry.update();
                // ToDo determine how many ticks represents lift down (left + right)
                if (pos> (target+10)) {

                    return true;
                } else {

                    return false;
                }
            }

        }
        public Action liftDown_PIDF() {
            return new LiftDown_PIDF();
        }
        public class LiftHangSample_PIDF implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=liftHangspecimen;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);
                telemetry.addData("Position ",pos);
                telemetry.update();
                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> (target+10) || (pos< target-10)) {

                    return true;
                } else {

                    return false;
                }
            }

        }
        public Action liftHangSample_PIDF() {
            return new LiftHangSample_PIDF();
        }
        public class LiftExtraBump_PIDF implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=liftExtrabump;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);
                telemetry.addData("Position ",pos);
                telemetry.update();
                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> (target+10) || pos< (target -10)) {

                    return true;
                } else {

                    return false;
                }
            }

        }
        public Action liftExtraBump_PIDF() {
            return new LiftExtraBump_PIDF();
        }
        public class LiftWall_PIDF implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=10;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);
                telemetry.addData("Position ",pos);
                telemetry.update();
                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> (target+10) || pos < (target -10)) {

                    return true;
                } else {

                    return false;
                }
            }

        }
        public Action liftWall_PIDF() {
            return new LiftWall_PIDF();
        }
    }
    public class Claw {
        private Servo claw;
        public Claw() {
            claw = hardwareMap.get(Servo.class, "claw");
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawOpen);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawClosed);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class ReleaseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawRelease);
                return false;
            }
        }

        public Action releaseClaw() {
            return new ReleaseClaw();
        }
        public class ClawWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawWall);
                return false;
            }
        }

        public Action clawWall() {
            return new ClawWall();
        }


    }
    public class Intake {
        private Servo intakePosition;
        public Intake() {
            intakePosition = hardwareMap.get(Servo.class, "intakePosition");
        }
        public class Intakeup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePosition.setPosition(intakeUp);
                return false;
            }
        }

        public Action intakeup() {
            return new Intakeup();
        }

        public class Intakedown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePosition.setPosition(intakeDown);
                return false;
            }
        }

        public Action intakedown() {
            return new Intakedown();
        }
    }
    public class Extendo {
        private Servo Lextendo;
        private Servo Rextendo;
        public Extendo() {
            Lextendo = hardwareMap.get(Servo.class, "Lextendo");
            Rextendo = hardwareMap.get(Servo.class, "Rextendo");
        }
        public class ExtendoOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Lextendo.setPosition(leftExtendoOut);
                Rextendo.setPosition(rightExtendoOut);
                return false;
            }
        }

        public Action extendoOut() {
            return new ExtendoOut();
        }

        public class ExtendoIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Lextendo.setPosition(leftExtendoIn);
                Rextendo.setPosition(rightExtendoIn);
                return false;
            }
        }

        public Action extednoIn() {
            return new ExtendoIn();
        }

        public class ExtendoHalf implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Lextendo.setPosition(leftExtendohalf);
                Rextendo.setPosition(rightExtendohalf);
                return false;
            }
        }

        public Action extednoHalf() {
            return new ExtendoHalf();
        }
    }
    public class Shoulder {
        private Servo leftShoulder;
        private Servo rightShoulder;
        public Shoulder() {
            leftShoulder = hardwareMap.get(Servo.class, "leftShoulder");
            rightShoulder = hardwareMap.get(Servo.class, "rightShoulder");
        }
        public class Shoulderwall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderWall);
                rightShoulder.setPosition(rightShoulderWall);
                return false;
            }
        }

        public Action shoulderWall() {
            return new Shoulderwall();
        }

        public class Shoulderbasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderbasket);
                rightShoulder.setPosition(rightShoulderbasket);
                return false;
            }
        }

        public Action shoulderbasket() {
            return new Shoulderbasket();
        }

        public class Shoulderintake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderintake);
                rightShoulder.setPosition(rightShoulderintake);
                return false;
            }
        }

        public Action shoulderintake() {
            return new Shoulderintake();
        }

        public class Shouldertransition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                return false;
            }
        }

        public Action shouldertransition() {
            return new Shouldertransition();
        }

        public class ShoulderoutOftheWay implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                return false;
            }
        }

        public Action shoulderoutOftheWay() {
            return new ShoulderoutOftheWay();
        }

        public class ShoulderHangSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderhangSpecimen);
                rightShoulder.setPosition(rightShoulderhangSpecimen);
                return false;
            }
        }

        public Action shoulderHangSpecimen() {
            return new ShoulderHangSpecimen();
        }
        public class ShoulderPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftShoulder.setPosition(leftShoulderpark);
                rightShoulder.setPosition(rightShoulderpark);
                return false;
            }
        }

        public Action shoulderpark() {
            return new ShoulderPark();
        }
    }
    public class Elbow {
        private Servo leftElbow;
        private Servo rightElbow;
        public Elbow() {
            leftElbow = hardwareMap.get(Servo.class, "leftElbow");
            rightElbow = hardwareMap.get(Servo.class, "rightElbow");
        }
        public class ElbowWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftElbow.setPosition(leftElbowWall);
                rightElbow.setPosition(rightElbowWall);
                return false;
            }
        }

        public Action elbowWall() {
            return new ElbowWall();
        }

        public class ElbowBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftElbow.setPosition(leftElbowbasket);
                rightElbow.setPosition(rightElbowbasket);
                return false;
            }
        }

        public Action elbowBasket() {
            return new ElbowBasket();
        }

        public class ElbowIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftElbow.setPosition(leftElbowintake);
                rightElbow.setPosition(rightElbowintake);
                return false;
            }
        }

        public Action elbowIntake() {
            return new ElbowIntake();
        }

        public class ElbowHang implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftElbow.setPosition(leftElbowhangSpecimen);
                rightElbow.setPosition(rightElbowhangSpecimen);
                return false;
            }
        }

        public Action elbowHang() {
            return new ElbowHang();
        }
        public class ElbowPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftElbow.setPosition(leftElbowpark);
                rightElbow.setPosition(rightElbowpark);
                return false;
            }
        }

        public Action elbowpark() {
            return new ElbowPark();
        }
    }
    public class IntakeWheel {
        public CRServo intakewheel;
        public IntakeWheel() {
            intakewheel = hardwareMap.get(CRServo.class, "intakeWheel");
        }
        public class WheelFoward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWheel.setPower(intakeWheelforward);
                return false;
            }
        }

        public Action wheelFoward() {
            return new WheelFoward();
        }

        public class WheelBackward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWheel.setPower(intakeWheelbackward);
                return false;
            }
        }

        public Action wheelBackward() {
            return new WheelBackward();
        }

        public class WheelOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWheel.setPower(intakeWheeloff);
                return false;
            }
        }

        public Action wheelOff() {
            return new WheelOff();
        }
    }

    public class BatteryVoltage {
        VoltageSensor batteryVoltage;

        public BatteryVoltage(HardwareMap hardwareMap) {
            batteryVoltage = hardwareMap.get(VoltageSensor.class, "Control Hub");


        }

        public class BatteryMonitor implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    double voltage = batteryVoltage.getVoltage();
                    telemetry.addData("Voltage ", voltage);
                    if (target < 100 && voltage < 10.5) {
                        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                }
                return true;
            }
        }

        public Action batteryMonitor() {
            return new BatteryMonitor();
        }
    }





    /*public void main() {
        extendoOut();
    }
    public class Lextendo {
        private Servo lextendo;

        public Lextendo(HardwareMap hardwareMap) {
            lextendo = hardwareMap.get(Servo.class, "Lextendo");
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
    }

     */


    //    Active intake function?
    public class Active_Intake {
        private Servo Lextendo;
        private Servo Rextendo;
        private CRServo intakewheel;
        private Servo intakePosition;
        private Servo rightShoulder;
        private Servo leftShoulder;
        private ElapsedTime runtime = new ElapsedTime();


        public Active_Intake() {
            Lextendo = hardwareMap.get(Servo.class, "Lextendo");
            Rextendo = hardwareMap.get(Servo.class, "Rextendo");
            intakewheel = hardwareMap.get(CRServo.class, "intakeWheel");
            intakePosition = hardwareMap.get(Servo.class, "intakePosition");
            intake1 = hardwareMap.get(ColorSensor.class, "intake1");
            leftShoulder = hardwareMap.get(Servo.class,"leftShoulder");
            rightShoulder = hardwareMap.get(Servo.class, "rightShoulder");

        }

        public class Active_IntakeOn implements Action {

            private boolean initialized = false;
            private double newLExtendoPosition = leftExtendohalf;
            private double newRExtendoPosition = rightExtendohalf;
            private boolean firstScan = true;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                Lextendo.setPosition(newLExtendoPosition);
                Rextendo.setPosition(newRExtendoPosition);
                red = intake1.red();
                blue = intake1.blue();
                green = intake1.green();
                Color.RGBToHSV(255 * red, 255 * green, 255 * blue, hsv);
                hue = hsv[0];
                saturation = hsv[1];
                value = hsv[2];
                if (hue >= 0 && hue < 30 && value > 200 || hue >= 330 && hue <= 360 && value > 200) {
                        intakecolorDetected = "Red";
                        intakecolorDetectedvalue = 1;
                }
                else if (hue > 30 && hue < 90 && value > 200) {
                        intakecolorDetected = "Yellow";
                        intakecolorDetectedvalue = 2;
                }
                else if (hue >= 210 && hue < 270 && value > 200) {
                    intakecolorDetected = "Blue";
                    intakecolorDetectedvalue = 3;
                }
                else {
                    intakecolorDetected = "None";
                    intakecolorDetectedvalue = 0;
                }
                telemetry.addData("Color Detected",intakecolorDetected);
                telemetry.addData("runtime",runtime.milliseconds());
                if (firstScan==true){
                    runtime.reset();
                    startTime = runtime.milliseconds();
                }
                //if (intakecolorDetectedvalue == 0&&newLExtendoPosition!=leftExtendoOut||intakecolorDetectedvalue == 3||intakecolorDetectedvalue==2&& newLExtendoPosition !=leftExtendoIn) {//color sensor sees no color or wrong color
                if(runtime.milliseconds()>startTime +3000){
                    intakewheel.setPower(intakeWheeloff);
                    intakePosition.setPosition(intakeUp);
                    sleep(100);
                    Lextendo.setPosition(leftExtendoIn);
                    Rextendo.setPosition(rightExtendoIn);
                    status=false;
                }
                else if(intakecolorDetectedvalue == 0&&newLExtendoPosition<=leftExtendoOut) { // no color and not extended
                        status=true;
                        intakewheel.setPower(intakeWheelforward);
                        newLExtendoPosition = newLExtendoPosition + 0.005;
                        newRExtendoPosition = newRExtendoPosition + 0.005;
                        if (firstScan){
                            sleep(100);
                            intakePosition.setPosition(intakeDown);
                            firstScan=false;
                        }
                    }
                    else if(intakecolorDetectedvalue==2&& newLExtendoPosition>=leftExtendoIn){
                        status =true;
                        intakewheel.setPower(intakeWheeloff);
                        intakePosition.setPosition(intakeUp);
                        newLExtendoPosition = newLExtendoPosition - 0.02;
                        newRExtendoPosition = newRExtendoPosition - 0.02;
                    }
                    else if (intakecolorDetectedvalue==3){   // wrong color
                        status=false;
                        intakewheel.setPower(intakeWheelbackward);
                    }
                    else if (intakecolorDetectedvalue==1&& newLExtendoPosition<=leftExtendoIn||intakecolorDetectedvalue==2 && newLExtendoPosition<=leftExtendoIn) {
                        status = false;
                    }

                    if(status==true){
                        return true;
                    }
                    else {
                        return false;
                    }



                    //if (intakecolorDetectedvalue == 1 || intakecolorDetectedvalue == 2 && newLExtendoPosition==leftExtendoIn || runtime.milliseconds()>startTime + 2000) {// sees correct color
                      //  intakePosition.setPosition(intakeUp);
                        //intakewheel.setPower(intakeWheeloff);
                        //Lextendo.setPosition(newLExtendoPosition);
                        //Rextendo.setPosition(newRExtendoPosition);

                    //}
                   /* else if (newLExtendoPosition == leftExtendoOut) {
                        intakewheel.setPower(intakeWheeloff);
                        intakePosition.setPosition(intakeUp);
                        sleep(100);
                        Lextendo.setPosition(leftExtendoIn);
                        Rextendo.setPosition(rightExtendoIn);
                    }

                    */


                }






            }


        public Action active_IntakeOn() {
            return new Active_IntakeOn();
        }

    }
}
