package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int OPTION = 2;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        if (OPTION == 1) {
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-23, -62, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-7, -35, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(-30, -40, Math.toRadians(180)), Math.toRadians(180))

                        .splineToLinearHeading(new Pose2d(-39, -25, Math.toRadians(180)), Math.toRadians(190))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToSplineHeading(new Pose2d(-49, -25, Math.toRadians(180)), Math.toRadians(190))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToSplineHeading(new Pose2d(-59, -25, Math.toRadians(180)), Math.toRadians(135))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
                        .build());
        } else if (OPTION == 2) {
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(23, -62, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(7, -35, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(0))

                        .splineToLinearHeading(new Pose2d(39, -25, Math.toRadians(0)), Math.toRadians(350))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(225))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(15))
                        .splineToSplineHeading(new Pose2d(49, -25, Math.toRadians(0)), Math.toRadians(350))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(30)), Math.toRadians(220))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToSplineHeading(new Pose2d(59, -25, Math.toRadians(0)), Math.toRadians(45))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(30)), Math.toRadians(220))
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(1)

                        .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
                        .build());
        } else {
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-58.1, -23, Math.toRadians(90)))
                        .strafeTo(new Vector2d(-58.1, 23))
                        .strafeTo(new Vector2d(-34.8, -23))
                        .strafeTo(new Vector2d(-34.8, 23))
                        
                        .strafeTo(new Vector2d(34.8, 23))

                        .strafeTo(new Vector2d(58.1, 23))
                        .strafeTo(new Vector2d(58.1, -23))
                        .strafeTo(new Vector2d(34.8, -23))
                        .strafeTo(new Vector2d(34.8,23))

                        .build());
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}