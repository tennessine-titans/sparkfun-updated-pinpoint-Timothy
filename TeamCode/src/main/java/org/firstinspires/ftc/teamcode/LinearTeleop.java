package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        while(opModeIsActive()){
            Lextendo.setPosition(leftExtendoOut);
            Rextendo.setPosition(rightExtendoOut);
            telemetry.addData("lExtendo",leftExtendoOut);
            telemetry.addData("rExtendo",rightExtendoOut);
            telemetry.update();
        }
    }
}
