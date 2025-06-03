package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test")
public class LinearTeleop extends Timothy{
    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    public void runOpMode() throws InterruptedException{
        waitForStart();
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
        while(opModeIsActive()){
            // D-pad left -  extendo out
            if (gamepad2.dpad_left){
                Lextendo.setPosition(leftExtendoOut);
                Rextendo.setPosition(rightExtendoOut);
            }
            // D-pad right -  extendo in
            else if (gamepad2.dpad_right){
                intakePosition.setPosition (intakeUp);
                sleep(100);
                Lextendo.setPosition (leftExtendoIn);
                Rextendo.setPosition (rightExtendoIn);
            }
            // Half extendo extended
            else if (gamepad2.dpad_up){
                Lextendo.setPosition (leftExtendohalf);
                Rextendo.setPosition (rightExtendohalf);
            }
            // Arms down to pick up out intake
            else if (gamepad2.dpad_down){
                claw.setPosition(clawOpen);
                rightShoulder.setPosition(rightShoulderintake);
                leftShoulder.setPosition(leftShoulderintake);
                rightElbow.setPosition(rightElbowintake);
                leftElbow.setPosition(leftElbowintake);
                //PID lifts down go here
            }
            // Get in position to hang specimen
            else if (gamepad2.cross){
                claw.setPosition(clawClosed);
                rightShoulder.setPosition(rightShoulderhangSpecimen);
                leftShoulder.setPosition(leftShoulderhangSpecimen);
                rightElbow.setPosition(rightElbowhangSpecimen);
                leftElbow.setPosition(leftElbowhangSpecimen);
            }
            else if (gamepad2.square){
                claw.setPosition(clawClosed);
                rightElbow.setPosition(rightElbowextraBump);
                leftElbow.setPosition(leftElbowextraBump);
            }
            // Tansfer sample from intake to claw and go up to put in bucket
            else if (gamepad2.triangle){
                claw.setPosition(clawOpen);
                intakePosition.setPosition (intakeUp);
                sleep(100);
                Lextendo.setPosition (leftExtendoIn);
                Rextendo.setPosition (rightExtendoIn);
                wait(1);
                leftElbow.setPosition(leftElbowintake);
                rightElbow.setPosition(rightElbowintake);
                sleep(1000);
                leftShoulder.setPosition(leftShoulderintake);
                rightShoulder.setPosition(rightShoulderintake);
                //PID code for lift position down will go here;
                claw.setPosition(clawClosed);
                //Lift PID to go up to basket goes here
                leftShoulder.setPosition (leftShoulderbasket);
                rightShoulder.setPosition(rightShoulderbasket);
                sleep(100);
                leftElbow.setPosition(leftElbowbasket);
                rightElbow.setPosition(rightElbowbasket);
            }
            // Go to position to pick up specimen off wall
            else if (gamepad2.circle){
                claw.setPosition(clawOpen);
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                sleep(100);
                leftElbow.setPosition(leftElbowWall);
                rightElbow.setPosition(rightElbowWall);
                sleep(100);
                rightShoulder.setPosition(rightShoulderWall);
                leftShoulder.setPosition(leftShoulderWall);
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
            }
            else if (gamepad1.left_bumper) {
                 intakePosition.setPosition(intakeUp);
                 intakeWheel.setPower(intakeWheeloff);
             }
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
