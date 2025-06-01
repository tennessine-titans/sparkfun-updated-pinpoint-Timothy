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
    protected double leftExtendoOut = 0.87;
    protected double rightExtendoOut = 0.72;
    // Define as servos
    public Servo Lextendo;
    public Servo Rextendo;
    public void intLextendo(){
        Lextendo = hardwareMap.get(Servo.class, "Lextendo");
        Lextendo.setDirection(Servo.Direction.REVERSE);
    }

    public void intRextendo(){
        Rextendo = hardwareMap.get(Servo.class, "Rextendo");
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







