package org.firstinspires.ftc.teamcode;
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
import com.qualcomm.robotcore.hardware.Servo;

@Config
public abstract class Timothy extends LinearOpMode {
    // Variables and actions used to set and run Timothys servo positions and motor positions.
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
    protected double rightShoulderintake = 0.46;
    protected double rightElbowintake = 0.31;
    protected double leftShoulderintake = 0.46;
    protected double leftElbowintake = 0.31;
    protected double rightShoulderbasket = 0.7;
    protected double leftShoulderbasket = 0.7;
    protected double rightElbowbasket = .89;
    protected double leftElbowbasket = .89;
    protected double clawClosed = .528;
    protected double clawOpen = .348;
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

    //    intakeSensor = hardwareMap.get(ColorSensor.class, "intakeSensor");
    //}
        //public void intclawSensor() {
           // clawSensor = hardwareMap.get(ColorSensor.class, "clawSensor");
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
            public double p = 0.01, i = 0.013, d = 0.0005, f = 0.06;
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
                    target=1490;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target-10) {
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

                // ToDo determine how many ticks represents lift down (left + right)
                if (pos< target+10) {
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
        public class LiftHangSample_PIDF implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target=350;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target-10 && pos< target+10) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
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
                    target=1000;
                    initialized = true;
                }

                double pos = lift1.getCurrentPosition();
                packet.put("liftpos",pos);

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target-10 && pos< target +10) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
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

                // ToDo determine how many ticks represents lift up (left + right)
                if (pos> target-10 && pos < target +10) {
                    telemetry.addData("Position ",pos);
                    telemetry.update();
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
    }
    public class Wheel {
        private CRServo wheel;
        public Wheel() {
            wheel = hardwareMap.get(CRServo.class, "intakeWheel");
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
    public void main() {
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
    }