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
                        .strafeToLinearHeading(new Vector2d(-7,30),3*Math.PI/2)
                        .setTangent(3*Math.PI/4)
                        .splineToConstantHeading(new Vector2d(-30, 40),Math.PI)
                        .splineToConstantHeading(new Vector2d(-38, 14),Math.PI)
                        .splineToConstantHeading(new Vector2d(-45, 58),Math.PI)
                        .splineToConstantHeading(new Vector2d(-48, 14),Math.PI)
                        .splineToConstantHeading(new Vector2d(-55, 58),Math.PI)
                        .splineToConstantHeading(new Vector2d(-58, 14),Math.PI)
                        .splineToConstantHeading(new Vector2d(-60, 16),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-55, 58),0)
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