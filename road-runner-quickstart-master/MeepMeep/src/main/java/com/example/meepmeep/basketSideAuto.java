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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, -61, Math.toRadians(-90)))
                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-66,-42,Math.toRadians(45)),Math.toRadians(135))
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(-56,-32,Math.toRadians(110)),Math.toRadians(70))
                        .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-66,-42,Math.toRadians(45)),Math.toRadians(135))
                        .setTangent(Math.toRadians(70))
                        .splineToLinearHeading(new Pose2d(-59, -32, Math.toRadians(110)),Math.toRadians(70))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-66,-42,Math.toRadians(45)),Math.toRadians(135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}