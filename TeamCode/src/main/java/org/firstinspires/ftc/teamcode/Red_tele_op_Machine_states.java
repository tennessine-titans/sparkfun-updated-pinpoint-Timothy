package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PwmControl;



@TeleOp(name = "Red_tele_op_Machine_states")
public class Red_tele_op_Machine_states extends Timothy {
    private ElapsedTime runtime = new ElapsedTime();

    private void outExtendo() {
        intakeDirectionForward = true;
        claw.setPosition(clawClosed);
        leftShoulder.setPosition(leftShoulderoutOftheWay);
        rightShoulder.setPosition(rightShoulderoutOftheWay);
        leftElbow.setPosition(leftElbowintake);
        rightElbow.setPosition(rightElbowintake);
        if (runtime.milliseconds() > startTime + 100) {
            Lextendo.setPosition(leftExtendoOut);
            Rextendo.setPosition(rightExtendoOut);
        }
        if(runtime.milliseconds()> startTime + 200) {
            lift1.setTargetPosition(0);
            lift2.setTargetPosition(0);
        }
    }

    private void inExtendo() {
        claw.setPosition(clawClosed);
        leftShoulder.setPosition(leftShoulderoutOftheWay);
        rightShoulder.setPosition(rightShoulderoutOftheWay);
        intakePosition.setPosition(intakeUp);
        if (runtime.milliseconds() > startTime + 100) {
            Lextendo.setPosition(leftExtendoIn);
            Rextendo.setPosition(rightExtendoIn);
        }
        intakeWheel.setPower(intakeWheeloff);
    }
    private void humanPlayertransfer(){
        if (runtime.milliseconds()< startTime + 100) {
            lift1.setTargetPosition(0);
            lift2.setTargetPosition(0);
            intakePosition.setPosition(intakeUp);
            rightShoulder.setPosition(rightShoulderoutOftheWay);
            leftShoulder.setPosition(leftShoulderoutOftheWay);
            claw.setPosition(clawOpen);
            step =1;
        }
         else if (runtime.milliseconds() > startTime + 100 && step==1) {
            Lextendo.setPosition(leftExtendoIn);
            Rextendo.setPosition(rightExtendoIn);
            step =2;
        }
        else if (runtime.milliseconds()> startTime + 550 && step ==2){
            rightShoulder.setPosition(rightShoulderintake);
            leftShoulder.setPosition(leftShoulderintake);
            step =3;
        }
        else if (runtime.milliseconds() > startTime + 850 && step== 3){
            claw.setPosition(clawClosed);
            if (lift1.getCurrentPosition()<10||lift2.getCurrentPosition()<10) {
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            step =4;
        }
        else if (runtime.milliseconds()>startTime +950 && step ==4){
            lift1.setTargetPosition(100);
            lift2.setTargetPosition(100);
            step =5;
        }
        else if (runtime.milliseconds()> startTime + 1100 && step == 5){
            rightShoulder.setPosition(rightShoulderspecimenTransition);
            leftShoulder.setPosition(leftShoulderspecimenTransition);
            step = 6;
        }
        else if (runtime.milliseconds()> startTime + 1400 && step ==6){
            rightElbow.setPosition(rightElbowWall);
            leftElbow.setPosition(leftElbowWall);
            step = 7;
        }
        else if (runtime.milliseconds()> startTime + 1700 && step == 7){
            rightShoulder.setPosition(rightShoulderWall);
            leftShoulder.setPosition(leftShoulderWall);
        }
    }
    private void armsOutofTheway(){
        lift1.setTargetPosition(10);
        lift2.setTargetPosition(10);
        claw.setPosition(clawClosed);
        rightShoulder.setPosition(rightShoulderoutOftheWay);
        leftShoulder.setPosition(leftShoulderoutOftheWay);
        if(runtime.milliseconds()> startTime +200) {
            rightElbow.setPosition(rightElbowintake);
            leftElbow.setPosition(leftElbowintake);
        }
    }
    private void specimenHang() {
        claw.setPosition(clawClosed);
        lift1.setTargetPosition(liftHangspecimen);
        lift2.setTargetPosition(liftHangspecimen);
        if (lift1.getCurrentPosition() > 150&&step==0) {
            rightShoulder.setPosition(rightShoulderspecimenTransition);
            leftShoulder.setPosition(leftShoulderspecimenTransition);
            startTime=runtime.milliseconds();
            step = 1;
        }
        if (runtime.milliseconds() > startTime + 200 && step == 1) {
            rightElbow.setPosition(rightElbowhangSpecimen);
            leftElbow.setPosition(leftElbowhangSpecimen);
            step = 2;
        }
        if (runtime.milliseconds() > startTime + 300 && step == 2){
            rightShoulder.setPosition(rightShoulderhangSpecimen);
            leftShoulder.setPosition(leftShoulderhangSpecimen);
    }
    }
    private void specimenExtrabump() {
        claw.setPosition(clawClosed);
        lift1.setTargetPosition(950);
        lift2.setTargetPosition(950);
    }
    private void basket() {
        if (runtime.milliseconds() < startTime + 100) {
            lift1.setTargetPosition(10);
            lift2.setTargetPosition(10);
            intakePosition.setPosition(intakeUp);
            rightShoulder.setPosition(rightShoulderoutOftheWay);
            leftShoulder.setPosition(leftShoulderoutOftheWay);
            claw.setPosition(clawOpen);
            step = 1;
        }
        if (runtime.milliseconds() > startTime + 100 && step == 1) {
            Lextendo.setPosition(leftExtendoIn);
            Rextendo.setPosition(rightExtendoIn);
            step = 2;
        }
        if (runtime.milliseconds() > startTime + 550 && step == 2) {
            rightShoulder.setPosition(rightShoulderintake);
            leftShoulder.setPosition(leftShoulderintake);
            step = 3;
        }
        if (runtime.milliseconds() > startTime + 850 && step == 3) {
            claw.setPosition(clawClosed);
            step = 4;
        }
        if (runtime.milliseconds() > startTime + 1000 && step == 4) {
            lift1.setTargetPosition(1500);
            lift2.setTargetPosition(1500);
            step = 5;
        }
        if (lift1.getCurrentPosition()>800&& step ==5) {
            leftShoulder.setPosition(leftShoulderbasket);
            rightShoulder.setPosition(rightShoulderbasket);
            step =6;
        }
        if(lift1.getCurrentPosition()>1200&& step ==6){
            leftElbow.setPosition(leftElbowbasket);
            rightElbow.setPosition(rightElbowbasket);
        }
    }
    private void grabFromwall() {
        lift1.setTargetPosition(10);
        lift2.setTargetPosition(10);
        if (runtime.milliseconds()<startTime +100){
            rightShoulder.setPosition(rightShoulderspecimenTransition);
            leftShoulder.setPosition(leftShoulderspecimenTransition);
            step = 1;
        }
        else if (runtime.milliseconds()> startTime + 300 && step ==1){
            rightElbow.setPosition(rightElbowWall);
            leftElbow.setPosition(leftElbowWall);
            step = 2;
        }
        else if (runtime.milliseconds()> startTime + 600 && step == 2){
            rightShoulder.setPosition(rightShoulderWall);
            leftShoulder.setPosition(leftShoulderWall);
        }
    }
    private void claw_Open(){
        claw.setPosition(clawOpen);
    }
    private void claw_Close(){
        claw.setPosition(clawClosed);
    }
    private void intake_Down(){
        // Set the intake position immediately upon bumper press.
        intakePosition.setPosition(intakeDown);
            if (intakecolorDetectedvalue == 1) {
                intakeWheel.setPower(0);// Stop
                intakePosition.setPosition(intakeUp);
                machineState =0;
            } else if (intakecolorDetectedvalue == 2) {
                intakeWheel.setPower(0); // Stop
                intakePosition.setPosition(intakeUp);
            } else if (intakecolorDetectedvalue == 3) {
                machineState=13;
                /*intakeWheel.setPower(-1); // Reverse
                if(scan){
                    intakePosition.setPosition(intakeUp);
                    sleep(50);
                    scan = false;
                }
                else if (!scan){
                    intakePosition.setPosition(intakeDown);
                    sleep(50);
                                }

                 */
            }
            else if(intakecolorDetectedvalue == 0) {
                intakeWheel.setPower(1);
                sleep(250);
        }
    }
    private void intake_Up(){
        intakePosition.setPosition(intakeUp);
        intakeWheel.setPower(intakeWheeloff);
            }
    private void intake_Down_Reverse() {
        // Set the intake position immediately upon bumper press.
        intakePosition.setPosition(intakeDown);
        intakeWheel.setPower(-1);
        sleep(250);
    }
    private void Stopped() {
    }
    private void liftReset() {
        lift1.setTargetPosition(-1000);
        lift2.setTargetPosition(-1000);
        sleep(250);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        machineState=0;
    }
    private void preHang() {
        lift1.setTargetPosition(liftpreHang);
        lift2.setTargetPosition(liftpreHang);
        sleep(500);
        leftShoulder.setPosition(leftShoulderpreHang);
        rightShoulder.setPosition(rightShoulderpreHang);
        sleep(100);
        leftElbow.setPosition(leftElbowpreHang);
        rightElbow.setPosition(rightElbowpreHang);
    }
    private void Hang() {
        lift1.setPower(0.9);
        lift1.setPower(0.9);
        PwmControl myPwmControlrightShoulderServo = (PwmControl) rightShoulder;
        myPwmControlrightShoulderServo.setPwmDisable();
        PwmControl myPwmControlleftShoulderServo = (PwmControl) leftShoulder;
        myPwmControlleftShoulderServo.setPwmDisable();
        PwmControl myPwmControlrightElbowServo = (PwmControl) rightElbow;
        myPwmControlrightElbowServo.setPwmDisable();
        PwmControl myPwmControlleftElbowServo = (PwmControl) leftElbow;
        myPwmControlleftElbowServo.setPwmDisable();
        PwmControl myPwmControlrightExtendoServo = (PwmControl) Rextendo;
        myPwmControlrightExtendoServo.setPwmDisable();
        PwmControl myPwmControlleftExtendoServo = (PwmControl) Lextendo;
        myPwmControlleftExtendoServo.setPwmDisable();
        PwmControl myPwmControlintakeServo = (PwmControl) intakePosition;
        myPwmControlintakeServo.setPwmDisable();
        PwmControl myPwmControlwheelServo = (PwmControl) intakeWheel;
        myPwmControlwheelServo.setPwmDisable();
        //((servoImplEx)rightShoulder).setPwmDiable();
        lift1.setTargetPosition(liftHang);
        lift2.setTargetPosition(liftHang);
        //sleep(100);
        //leftShoulder.setPosition(leftShoulderHang);
        //rightShoulder.setPosition(rightShoulderHang);
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
        intleftLight();
        intrightLight();
        //intintake1();
        //intclawSensor();
        //int red = intake1.red();
        //int blue = intake1.blue();
        //int green = intake1.green();
        //Lextendo.setPosition(leftExtendoIn);
        //Rextendo.setPosition(rightExtendoIn);
        //rightShoulder.setPosition(rightShoulderintake);
        //leftShoulder.setPosition(leftShoulderintake);
        //leftElbow.setPosition(leftElbowintake);
        //rightElbow.setPosition(rightElbowintake);
        //claw.setPosition(clawClosed);
        //intakePosition.setPosition(intakeUp);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        intakePosition.setPosition(intakeUp);
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
        //lift1.setPower(-0.5);
        //lift2.setPower(-0.5);
        //sleep(250);
        //lift1.setPower(0);
        //lift2.setPower(0);

        while (opModeIsActive()) {
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(.7);
            lift2.setPower(.7);
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
                rightLight.setPosition(.279);
            } else if (hue > 30 && hue < 90 && value > 200) {
                intakecolorDetected = "Yellow";
                intakecolorDetectedvalue = 2;
                rightLight.setPosition(.388);
            } else if (hue >= 210 && hue < 270 && value > 200) {
                intakecolorDetected = "Blue";
                intakecolorDetectedvalue = 3;
                rightLight.setPosition(.611);
            } else {
                intakecolorDetected = "None";
                intakecolorDetectedvalue =0;
                rightLight.setPosition(0.0);
            }
            if(intakeWheel.getPower()>0){
                leftLight.setPosition(0.5);
            }
            else if (intakeWheel.getPower()<0){
                leftLight.setPosition(0.333);
            }
            else if(intakeWheel.getPower()==0){
                leftLight.setPosition(0.0);
            }
            // D-pad left -  extendo out
            if (gamepad2.dpad_left) {
                step = 0;
                machineState = 1;
                startTime = runtime.milliseconds();
            }
            // D-pad right -  extendo in
            else if (gamepad2.dpad_right) {
                step = 0;
                machineState =2;
                startTime = runtime.milliseconds();
            }
            // Transfer to human player
            else if (gamepad2.dpad_up) {
                step = 0;
                machineState=3;
                startTime = runtime.milliseconds();
            }
            // Arms down to pick up out intake
            else if (gamepad2.dpad_down) {
                step = 0;
                machineState =4;
                startTime = runtime.milliseconds();
            }
            // Get in position to hang specimen
            else if (gamepad2.cross) {
                step = 0;
                machineState = 5;
                startTime = runtime.milliseconds();


            } else if (gamepad2.square) {
                step = 0;
                machineState =6;
                startTime = runtime.milliseconds();
            }
            // Tansfer sample from intake to claw and go up to put in bucket
            else if (gamepad2.triangle) {
                step = 0;
                machineState = 7;
                startTime = runtime.milliseconds();
            }
            // Go to position to pick up specimen off wall
            else if (gamepad2.circle) {
                step = 0;
                machineState = 8;
                startTime = runtime.milliseconds();
            }
            //open claw
            else if (gamepad2.right_bumper) {
                step = 0;
                machineState =9;
                startTime = runtime.milliseconds();
            }
            //closed claw
            else if (gamepad2.left_bumper) {
                step = 0;
                machineState =10;
                startTime = runtime.milliseconds();
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
            // intake down and collecting
            if (gamepad1.right_bumper) {
                step = 0;
                if(intakeDirectionForward){ //intake direction forward = TRUE
                    machineState = 13;
                    intakeDirectionForward=false;
                } else {       //intake direction forward = FALSE (reversed direction)
                    machineState = 11;
                    startTime = runtime.milliseconds();
                    intakeDirectionForward=true;
                }
            }

            // intake up
            else if (gamepad1.left_bumper) {
                step = 0;
                machineState=12;
                startTime = runtime.milliseconds();

            }
            else if (gamepad1.dpad_left) {
                step = 0;
                machineState=14;
            }
             else if (gamepad1.cross) {
                step = 0;
                machineState=15;
            }
            else if (gamepad1.triangle) {
                step = 0;
                machineState=16;
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
            if (machineState == 0) {
                Stopped();
            }
            else if (machineState == 1){
                outExtendo();
            }
            else if (machineState == 2){
                inExtendo();
            }
            else if (machineState == 3){
                humanPlayertransfer();
            }
            else if (machineState == 4){
                armsOutofTheway();
            }
            else if (machineState == 5){
                specimenHang();
            }
            else if (machineState == 6){
                specimenExtrabump();
            }
            else if (machineState == 7){
                basket();
            }
            else if (machineState == 8){
                grabFromwall();
            }
            else if (machineState == 9){
                claw_Open();
            }
            else if (machineState == 10){
                claw_Close();
            }
            else if (machineState == 11){
                intake_Down();
            }
            else if (machineState == 12){
                intake_Up();
            }
            else if (machineState == 13){
                intake_Down_Reverse();
            }
            else if (machineState == 14){
                liftReset();
            }
            else if (machineState == 15){
                preHang();
            }
            else if (machineState == 16){
                Hang();
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
            telemetry.addData("step", step);
            telemetry.update();


        }
    }
}






