package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class specSideAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -30, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45,-50, Math.toRadians(0)), Math.toRadians(-90))
                        .waitSeconds(.1)
                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53,-10,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(.3)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(54,-50, Math.toRadians(0)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(54,-40, Math.toRadians(90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(54, -53, Math.toRadians(90)), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}