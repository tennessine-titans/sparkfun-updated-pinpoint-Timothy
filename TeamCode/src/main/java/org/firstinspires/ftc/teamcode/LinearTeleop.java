package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "test")
public class LinearTeleop extends Timothy{


    @Override
    public void runOpMode() {
       // waitForStart();
        intLextendo();
        intRextendo();
        intintakePosition();
        intintakeWheel();
        intleftElbow();
        intrightElbow();
        intleftShoulder();
        intrightShoulder();
        intlift1();
        intlift2();
        intclaw();
        //intintakeSensor();
        //intclawSensor();
        //int red = intakeSensor.red();
        //int blue = intakeSensor.blue();
        //int green = intakeSensor.green();
        Lextendo.setPosition(leftExtendoIn);
        Rextendo.setPosition(rightExtendoIn);
        rightShoulder.setPosition(rightShoulderintake);
        leftShoulder.setPosition(leftShoulderintake);
        leftElbow.setPosition(leftElbowintake);
        rightElbow.setPosition(rightElbowintake);
        claw.setPosition(clawOpen);
        intakePosition.setPosition(intakeUp);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(.5);
            lift2.setPower(.5);
            // D-pad left -  extendo out
            if (gamepad2.dpad_left){
                claw.setPosition(clawOpen);
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                sleep(100);
                Lextendo.setPosition(leftExtendoOut);
                Rextendo.setPosition(rightExtendoOut);
            }
            // D-pad right -  extendo in
            else if (gamepad2.dpad_right){
                claw.setPosition(clawOpen);
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                intakePosition.setPosition (intakeUp);
                sleep(100);
                Lextendo.setPosition (leftExtendoIn);
                Rextendo.setPosition (rightExtendoIn);
            }
            // Transfer to human player
            else if (gamepad2.dpad_up){
                intakePosition.setPosition(intakeUp);
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                claw.setPosition(clawOpen);
                Lextendo.setPosition(leftExtendoIn);
                Rextendo.setPosition(rightExtendoIn);
                sleep(100);
                rightShoulder.setPosition(rightShoulderintake);
                leftShoulder.setPosition(leftShoulderintake);
                claw.setPosition(clawClosed);
                sleep(100);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                sleep(100);
                rightElbow.setPosition(rightElbowWall);
                leftElbow.setPosition(leftElbowWall);
                sleep(100);
                rightShoulder.setPosition(rightShoulderWall);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
            }
            // Arms down to pick up out intake
            else if (gamepad2.dpad_down){
                claw.setPosition(clawOpen);
                rightShoulder.setPosition(rightShoulderintake);
                leftShoulder.setPosition(leftShoulderintake);
                rightElbow.setPosition(rightElbowintake);
                leftElbow.setPosition(leftElbowintake);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
            }
            // Get in position to hang specimen
            else if (gamepad2.cross){
                lift1.setTargetPosition(650);
                claw.setPosition(clawClosed);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                sleep(100);
                rightElbow.setPosition(rightElbowhangSpecimen);
                leftElbow.setPosition(leftElbowhangSpecimen);
                sleep(100);
                rightShoulder.setPosition(rightShoulderhangSpecimen);
                leftShoulder.setPosition(leftShoulderhangSpecimen);

            }
            else if (gamepad2.square){
                claw.setPosition(clawClosed);
                lift1.setTargetPosition(950);
                lift2.setTargetPosition(950);
            }
            // Tansfer sample from intake to claw and go up to put in bucket
            else if (gamepad2.triangle){
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                claw.setPosition(clawOpen);
                intakePosition.setPosition (intakeUp);
                sleep(100);
                Lextendo.setPosition (leftExtendoIn);
                Rextendo.setPosition (rightExtendoIn);
                sleep(1);
                leftElbow.setPosition(leftElbowintake);
                rightElbow.setPosition(rightElbowintake);
                sleep(1000);
                leftShoulder.setPosition(leftShoulderintake);
                rightShoulder.setPosition(rightShoulderintake);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
                claw.setPosition(clawClosed);
                lift1.setTargetPosition(1500);
                lift2.setTargetPosition(1500);
                leftShoulder.setPosition (leftShoulderbasket);
                rightShoulder.setPosition(rightShoulderbasket);
                sleep(100);
                leftElbow.setPosition(leftElbowbasket);
                rightElbow.setPosition(rightElbowbasket);
            }
            // Go to position to pick up specimen off wall
            else if (gamepad2.circle){
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                sleep(100);
                leftElbow.setPosition(leftElbowWall);
                rightElbow.setPosition(rightElbowWall);
                sleep(100);
                rightShoulder.setPosition(rightShoulderWall);
                leftShoulder.setPosition(leftShoulderWall);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
            }
            //open claw
            else if (gamepad2.right_bumper){
                claw.setPosition(clawOpen);
            }
            //closed claw
            else if (gamepad2.left_bumper){
                claw.setPosition(clawClosed);
            }
            // Right bumper - Intake down
             if (gamepad1.right_bumper){
                intakePosition.setPosition (intakeDown);
                intakeWheel.setPower (intakeWheelforward);
                /* if (isRed || isYellow) {
                     intakeWheel.setPower(0); // Stop
                 } else if (isBlue) {
                     intakeWheel.setPower(-1); // Reverse
                 } else {
                     intakeWheel.setPower(1); // Forward
                 }

                 */
            }
            else if (gamepad1.left_bumper) {
                 intakePosition.setPosition(intakeUp);
                 intakeWheel.setPower(intakeWheeloff);
             }
            telemetry.addData("lift1",lift1.getCurrentPosition());
            telemetry.addData("lift2",lift2.getCurrentPosition());
            telemetry.update();

            /*
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();
            /*
            Lextendo.setPosition(leftExtendoOut);
            Rextendo.setPosition(rightExtendoOut);
            double le = Lextendo.getPosition();
            double re = Rextendo.getPosition();
            telemetry.addData("lExtendo",le);
            telemetry.addData("rExtendo",re);
            telemetry.update();

             */
        }


    }
}
