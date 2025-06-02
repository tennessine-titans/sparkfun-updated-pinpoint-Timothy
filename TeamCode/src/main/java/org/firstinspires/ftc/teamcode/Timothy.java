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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Action;
@Config
public abstract class Timothy extends LinearOpMode {
    // Variables and actions used to set and run Timothys servo postions and motor positions.
    //Set default values;
    protected float intakeDown = 0;// to pick up sample
    protected float intakeUp = 1; // postiton when extendo is retracted
    protected double intakeWheelforward = 1;
    protected double intakeWheelbackward = -1;
    protected double intakeWheeloff = 0;
    protected double leftExtendoOut = 0.87;
    protected double rightExtendoOut = 0.72;
    // Define as servos
    protected double leftExtendoIn = 0.1;
    protected double rightExtendoIn= 0;
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
    protected double rightShoulderhangSpecimen = .675;
    protected double leftShoulderhangSpecimen = .675;
    protected double leftElbowhangSpecimen = .158;
    protected double rightElbowhangSpecimen = .158;
    protected double leftElbowextraBump = .183;
    protected double rightElbowextraBump =.183;


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
    public void intLextendo(){
        Lextendo = hardwareMap.get(Servo.class, "Lextendo");
        Lextendo.setDirection(Servo.Direction.REVERSE);
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
    }
    public void intlift2(){
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
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

    }

 */
}







