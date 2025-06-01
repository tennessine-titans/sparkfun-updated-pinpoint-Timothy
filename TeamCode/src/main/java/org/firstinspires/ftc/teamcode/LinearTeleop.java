package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test")
public class LinearTeleop extends Timothy{
    @Override
    public void runOpMode() throws InterruptedException{
        intLextendo();
        while(opModeIsActive()){
            Lextendo.setPosition(leftExtendoOut);
            Rextendo.setPosition(rightExtendoOut);
        }
    }
}