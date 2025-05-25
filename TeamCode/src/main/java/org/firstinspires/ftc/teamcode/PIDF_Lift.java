package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;



@Config
@TeleOp
public class PIDF_Lift extends OpMode {
    private PIDController controller;

    public static double p=0,i=0,d=0;
    public static double f= 0;
    public static int target = 0;
    private DcMotorEx lift1;
    private DcMotorEx lift2;

    @Override
    public void init(){
    controller = new PIDController(p,i,d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    lift1 = hardwareMap.get(DcMotorEx.class,"lift1");
    lift2= hardwareMap.get(DcMotorEx.class, "lift2");
    lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    lift1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int liftPos = (lift1.getCurrentPosition()+lift2.getCurrentPosition())/2;
        double pid = controller.calculate(liftPos,target);
        double ff = f;
        double power = pid + ff;

        lift1.setPower(power);
        lift2.setPower(power);

        telemetry.addData ( "lifts ",liftPos);
        telemetry.addData("target ", target);
        telemetry.addData("power", power);
        telemetry.update();
    }

}
// p = 0.01 d = 0.0004 f =0.06 i =0.013