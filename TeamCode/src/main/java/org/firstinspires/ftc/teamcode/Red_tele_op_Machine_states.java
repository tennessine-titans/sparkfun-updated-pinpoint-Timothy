package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "Red_tele_op_Machine_states")
public class Red_tele_op_Machine_states extends Timothy {
    private ElapsedTime runtime = new ElapsedTime();

    private void outExtendo() {
        claw.setPosition(clawOpen);
        leftShoulder.setPosition(leftShoulderoutOftheWay);
        rightShoulder.setPosition(rightShoulderoutOftheWay);
        if (runtime.milliseconds() > startTime + 100) {
            Lextendo.setPosition(leftExtendoOut);
            Rextendo.setPosition(rightExtendoOut);
        }
    }

    private void inExtendo() {
        claw.setPosition(clawOpen);
        leftShoulder.setPosition(leftShoulderoutOftheWay);
        rightShoulder.setPosition(rightShoulderoutOftheWay);
        intakePosition.setPosition(intakeUp);
        if (runtime.milliseconds() > startTime + 100) {
            Lextendo.setPosition(leftExtendoIn);
            Rextendo.setPosition(rightExtendoIn);
        }
    }
    private void humanPlayertransfer(){
        if (runtime.milliseconds()< startTime + 100) {
            lift1.setTargetPosition(10);
            lift2.setTargetPosition(10);
            intakePosition.setPosition(intakeUp);
            rightShoulder.setPosition(rightShoulderoutOftheWay);
            leftShoulder.setPosition(leftShoulderoutOftheWay);
            claw.setPosition(clawOpen);
        }
        if (runtime.milliseconds() > startTime + 100) {
            Lextendo.setPosition(leftExtendoIn);
            Rextendo.setPosition(rightExtendoIn);
        }
        if (runtime.milliseconds()> startTime + 300){
            rightShoulder.setPosition(rightShoulderintake);
            leftShoulder.setPosition(leftShoulderintake);
            claw.setPosition(clawClosed);
        }
        if (runtime.milliseconds()> startTime + 3000){
            rightShoulder.setPosition(rightShoulderspecimenTransition);
            leftShoulder.setPosition(leftShoulderspecimenTransition);
        }
        if (runtime.milliseconds()> startTime + 4000){
            rightElbow.setPosition(rightElbowWall);
            leftElbow.setPosition(leftElbowWall);
        }
        if (runtime.milliseconds()> startTime + 5000){
            rightShoulder.setPosition(rightShoulderWall);
            leftShoulder.setPosition(leftShoulderWall);
        }
    }

    @Override
    public void runOpMode() {
        runtime.reset();
        intake1 = hardwareMap.get(ColorSensor.class, "intake1");
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
        //intintake1();
        //intclawSensor();
        //int red = intake1.red();
        //int blue = intake1.blue();
        //int green = intake1.green();
        Lextendo.setPosition(leftExtendoIn);
        Rextendo.setPosition(rightExtendoIn);
        rightShoulder.setPosition(rightShoulderintake);
        leftShoulder.setPosition(leftShoulderintake);
        leftElbow.setPosition(leftElbowintake);
        rightElbow.setPosition(rightElbowintake);
        claw.setPosition(clawClosed);
        intakePosition.setPosition(intakeUp);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
         /*abstract class Colorsensor extends LinearOpMode{
            NormalizedColorSensor intake1;
            int scaleFactor = 1000;
            public Colorsensor(NormalizedColorSensor intake1){this.intake1= intake1;}
            private float[] ColorsensorCheck(){
                int red = Math.round(intake1.getNormalizedColors().red*255*scaleFactor);
                int green = Math.round(intake1.getNormalizedColors().green*255*scaleFactor);
                int blue = Math.round(intake1.getNormalizedColors().blue*255*scaleFactor);
                float[] hsv = new float[3];
                Color.RGBToHSV(red, green, blue, hsv);
                return hsv;
            }
        }*/

        while (opModeIsActive()) {
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(.5);
            lift2.setPower(.5);
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
            } else if (hue > 30 && hue < 90 && value > 200) {
                intakecolorDetected = "Yellow";
                intakecolorDetectedvalue = 2;
            } else if (hue >= 210 && hue < 270 && value > 200) {
                intakecolorDetected = "Blue";
                intakecolorDetectedvalue = 3;
            } else {
                intakecolorDetected = "None";
            }

            // D-pad left -  extendo out
            if (gamepad2.dpad_left) {
                machineState = 1;
                startTime = runtime.milliseconds();
            }
            // D-pad right -  extendo in
            else if (gamepad2.dpad_right) {
                machineState =2;
                startTime = runtime.milliseconds();
            }
            // Transfer to human player
            else if (gamepad2.dpad_up) {
                machineState=3;
                startTime = runtime.milliseconds();
            }
            // Arms down to pick up out intake
            else if (gamepad2.dpad_down) {
                claw.setPosition(clawOpen);
                rightShoulder.setPosition(rightShoulderintake);
                leftShoulder.setPosition(leftShoulderintake);
                rightElbow.setPosition(rightElbowintake);
                leftElbow.setPosition(leftElbowintake);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
                claw.setPosition(clawClosed);
            }
            // Get in position to hang specimen
            else if (gamepad2.cross) {
                lift1.setTargetPosition(650);
                lift2.setTargetPosition(650);
                claw.setPosition(clawClosed);
                rightShoulder.setPosition(rightShoulderspecimenTransition);
                leftShoulder.setPosition(leftShoulderspecimenTransition);
                sleep(100);
                rightElbow.setPosition(rightElbowhangSpecimen);
                leftElbow.setPosition(leftElbowhangSpecimen);
                sleep(100);
                rightShoulder.setPosition(rightShoulderhangSpecimen);
                leftShoulder.setPosition(leftShoulderhangSpecimen);

            } else if (gamepad2.square) {
                claw.setPosition(clawClosed);
                lift1.setTargetPosition(950);
                lift2.setTargetPosition(950);
            }
            // Tansfer sample from intake to claw and go up to put in bucket
            else if (gamepad2.triangle) {
                rightShoulder.setPosition(rightShoulderoutOftheWay);
                leftShoulder.setPosition(leftShoulderoutOftheWay);
                claw.setPosition(clawOpen);
                intakePosition.setPosition(intakeUp);
                sleep(100);
                Lextendo.setPosition(leftExtendoIn);
                Rextendo.setPosition(rightExtendoIn);
                sleep(1);
                leftElbow.setPosition(leftElbowintake);
                rightElbow.setPosition(rightElbowintake);
                sleep(1000);
                leftShoulder.setPosition(leftShoulderintake);
                rightShoulder.setPosition(rightShoulderintake);
                lift1.setTargetPosition(10);
                lift2.setTargetPosition(10);
                sleep(2000);
                claw.setPosition(clawClosed);
                sleep(200);
                lift1.setTargetPosition(1500);
                lift2.setTargetPosition(1500);
                leftShoulder.setPosition(leftShoulderbasket);
                rightShoulder.setPosition(rightShoulderbasket);
                sleep(100);
                leftElbow.setPosition(leftElbowbasket);
                rightElbow.setPosition(rightElbowbasket);
            }
            // Go to position to pick up specimen off wall
            else if (gamepad2.circle) {
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
            else if (gamepad2.right_bumper) {
                claw.setPosition(clawOpen);
            }
            //closed claw
            else if (gamepad2.left_bumper) {
                claw.setPosition(clawClosed);
            }
            // Right bumper - Intake down
            /*if (gamepad1.right_bumper) {
                intakePosition.setPosition(intakeDown);
                intakeWheel.setPower(1);
                if (intakecolorDetectedvalue == 1) {
                    intakeWheel.setPower(0); // Stop
                } else if (intakecolorDetectedvalue == 2) {
                    intakeWheel.setPower(0); // stop
                } else if (intakecolorDetectedvalue == 3) {
                    intakeWheel.setPower(-1); // Reverse
                }


             */
            if (gamepad1.right_bumper) {
                // Set the intake position immediately upon bumper press.
                intakePosition.setPosition(intakeDown);

                // Check if the intake wheel is currently reversing.
                // If it is, pressing the bumper again will force it to spin forward.
                if (intakeWheel.getPower() < 0) {
                    intakeWheel.setPower(1); // Force forward
                } else {
                    // If the wheel was not reversing (i.e., stopped or already forward),
                    // then apply the color detection logic as before.
                    if (intakecolorDetectedvalue == 1) {
                        intakeWheel.setPower(0); // Stop
                    } else if (intakecolorDetectedvalue == 2) {
                        intakeWheel.setPower(0); // Stop
                    } else if (intakecolorDetectedvalue == 3) {
                        intakeWheel.setPower(-1); // Reverse
                    } else {
                        intakeWheel.setPower(1);
                    }
                }
            }


            if (gamepad1.left_bumper) {
                intakePosition.setPosition(intakeUp);
                intakeWheel.setPower(intakeWheeloff);


            }


            DcMotor leftFrontdrive = hardwareMap.get(DcMotor.class, "leftFront");
            DcMotor leftBackdrive = hardwareMap.get(DcMotor.class, "leftBack");
            DcMotor rightFrontdrive = hardwareMap.get(DcMotor.class, "rightFront");
            DcMotor rightBackdrive = hardwareMap.get(DcMotor.class, "rightBack");
            leftFrontdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackdrive.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();

            {
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                double max = Math.max(
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
                );

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                leftFrontdrive.setPower(leftFrontPower);
                rightFrontdrive.setPower(rightFrontPower);
                leftBackdrive.setPower(leftBackPower);
                rightBackdrive.setPower(rightBackPower);

            }
            if (machineState == 1){
                outExtendo();
            }
            else if (machineState == 2){
                inExtendo();
            }
            else if (machineState == 3){
                humanPlayertransfer();
            }






            /*

             */

            telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("hue", hue);
            telemetry.addData("saturation", saturation);
            telemetry.addData("value", value);
            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("green", green);
            telemetry.addData("color detected", intakecolorDetected);
            //Lextendo.setPosition(leftExtendoOut);
            //Rextendo.setPosition(rightExtendoOut);
            double le = Lextendo.getPosition();
            double re = Rextendo.getPosition();
            telemetry.addData("lExtendo", le);
            telemetry.addData("rExtendo", re);
            telemetry.addData("time", runtime);
            telemetry.update();


        }
    }
}






