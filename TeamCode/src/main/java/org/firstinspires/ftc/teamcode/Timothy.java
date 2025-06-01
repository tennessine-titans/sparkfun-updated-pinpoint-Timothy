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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Action;

public class Timothy {
    // Variables and actions used to set and run Timothys servo postions and motor positions.
    //Set default values;
    protected double intakeDown = 0;// to pick up sample
    protected double intakeUp = 1; // postiton when extendo is retracted
    protected double leftExtendoOut = 0.87;
    protected double rightExtendoOut = 0.72;
    // Define as servos
    //private Servo Lextendo;
    //private Servo Rextendo;
    Servo Lextendo = HardwareMap.get(Servo.class,  "Lextendo");
    Servo Rextendo = HardwareMap.get(Servo.class,  "Rextendo");
    public void extendoOut(){
        Lextendo.setposition (leftExtendoOut);
        Rextendo.setposition (rightExtendoOut);
    }
        public void main() {
        extendoOut();
        }
}





