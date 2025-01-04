package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Samples {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(700);
		int OPTION = 1;

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.PI, Math.PI, 13)
				.build();

		if (OPTION == 1) {
			Action firstSample = myBot.getDrive().actionBuilder(new Pose2d(-23, -62, Math.toRadians(0)))
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(-40, -55), Math.toRadians(180))
				.splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(180))
				.build();

			Action basket = myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(45)))
				.strafeTo(new Vector2d(-55, -55))
				.build();
			
			Action intakeFirst = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
				.splineToLinearHeading(new Pose2d(-36, -25, Math.toRadians(180)), Math.toRadians(90))
				.build();

			Action depositFirst = myBot.getDrive().actionBuilder(new Pose2d(-36, -25, Math.toRadians(180)))
				.splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(225))
				.build();

			Action intakeSecond = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
				.splineToLinearHeading(new Pose2d(-48, -25, Math.toRadians(180)), Math.toRadians(180))
				.build();

			Action depositSecond = myBot.getDrive().actionBuilder(new Pose2d(-48, -25, Math.toRadians(180)))
				.splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270))
				.build();

			Action intakeThird = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
				.splineToLinearHeading(new Pose2d(-59, -25, Math.toRadians(180)), Math.toRadians(180))
				.build();

			Action depositThird = myBot.getDrive().actionBuilder(new Pose2d(-59, -25, Math.toRadians(180)))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270))
				.build();

			Action park = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
				.splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
				.build();

			myBot.runAction(new SequentialAction(
				firstSample,
				basket,
				intakeFirst,
				depositFirst,
				basket,
				intakeSecond,
				depositSecond,
				basket,
				intakeThird,
				depositThird,
				basket,
				park
			));
		} else if (OPTION == 2) {
			Action first = myBot.getDrive().actionBuilder(new Pose2d(23, -62, Math.toRadians(90)))
				.splineToSplineHeading(new Pose2d(7, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();

			Action second = myBot.getDrive().actionBuilder(new Pose2d(7, -35, Math.toRadians(270)))
				.splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(0))
				.splineToLinearHeading(new Pose2d(39, -25, Math.toRadians(0)), Math.toRadians(350))
				.build();

			Action third = myBot.getDrive().actionBuilder(new Pose2d(39, -25, Math.toRadians(0)))
				.setReversed(true)
				.splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(225))
				.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
				.build();

			Action fourth = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))

				.splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)), Math.toRadians(15))
				.splineToSplineHeading(new Pose2d(49, -25, Math.toRadians(0)), Math.toRadians(350))
				.build();

			Action fifth = myBot.getDrive().actionBuilder(new Pose2d(49, -25, Math.toRadians(0)))
				.setReversed(true)
				.splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(30)), Math.toRadians(220))
				.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
				.build();

			Action sixth = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))

				.splineToSplineHeading(new Pose2d(59, -25, Math.toRadians(0)), Math.toRadians(45))
				.build();

			Action seventh = myBot.getDrive().actionBuilder(new Pose2d(59, -25, Math.toRadians(0)))
				.setReversed(true)
				.splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(30)), Math.toRadians(220))
				.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
				.build();

			Action eighth = myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))

				.splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
				.build();

			myBot.runAction(new SequentialAction(
				first,
				second,
				third,
				fourth,
				fifth,
				sixth,
				seventh,
				eighth
			));
		} else if (OPTION == 3) {
			myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
					.turn(99999)
					.build());
		} else if (OPTION == 4) {
			// Spline test in RR tuning
			myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
				.splineTo(new Vector2d(30, 30), Math.PI / 2)
				.splineTo(new Vector2d(0, 60), Math.PI)

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