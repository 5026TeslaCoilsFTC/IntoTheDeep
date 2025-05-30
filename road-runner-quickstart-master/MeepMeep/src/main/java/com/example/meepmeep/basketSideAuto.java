package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class basketSideAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39, -61, Math.toRadians(0)))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-55, -60, Math.toRadians( 45)), Math.toRadians(180))
                //collect2
                        .waitSeconds(.2)
                        .splineToLinearHeading(new Pose2d(-49,-50, Math.toRadians(90)), Math.toRadians( 90))
                // place 2
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-55,-60,Math.toRadians(45)), Math.toRadians(-135))
                //collect3
                .waitSeconds(.2)
                        .setTangent( Math.toRadians(45))

                .splineToLinearHeading(new Pose2d(-57,-50, Math.toRadians(90)), Math.toRadians( 90))
                // place 3
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-55,-60,Math.toRadians(45)), Math.toRadians(-135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}