package com.example.meepmeeptesting;
//import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .build();
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-9, 64, 3*Math.PI/2))
                        // go to submersible
                        .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                        // drvie to first sample
                        .setTangent(5*Math.PI/8)
                        .splineToConstantHeading(new Vector2d(-30, 40),Math.PI)
                        .splineToConstantHeading(new Vector2d(-38, 14),Math.PI)
                        //push sample to wall
                        .splineToConstantHeading(new Vector2d(-45, 48),Math.PI)
                        // go behind second sample
                        .splineToConstantHeading(new Vector2d(-50, 14),Math.PI)
                        //push sample to wall
                        .splineToConstantHeading(new Vector2d(-55, 48),Math.PI)
                        //go behind third sample
                        .splineToConstantHeading(new Vector2d(-56, 14),Math.PI)
                        .splineToConstantHeading(new Vector2d(-62, 14),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-55, 48),0)
                        //Pick up second sample off the wall
                        .splineToConstantHeading(new Vector2d(-40, 60),Math.PI/2)
                        //Hang second specimen
                        .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                        // Get third specimen off the wall
                        .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                        //hang third specimen
                        .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                        // Get fourth specimen off the wall
                        .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                        //hang fourth specimen
                        .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                        // Get fifth specimen off the wall
                        .strafeToLinearHeading(new Vector2d(-40, 60),3*Math.PI/2)
                        //hang fifth specimen
                        .strafeToLinearHeading(new Vector2d(-3,30),3*Math.PI/2)
                        .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\FTC21457\\Pictures\\field-2024-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
        //meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}