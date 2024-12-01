package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Specimens {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(700);
		int OPTION = 2;

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(100, 100, Math.toRadians(18000), Math.toRadians(180), 13)
				.build();

		if (OPTION == 1) {
			Action first = myBot.getDrive().actionBuilder(new Pose2d(24, -62, Math.toRadians(270)))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(7, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();

			Action second = myBot.getDrive().actionBuilder(new Pose2d(7, -35, Math.toRadians(270)))
				.splineToSplineHeading(new Pose2d(36, -28, Math.toRadians(180)), Math.toRadians(100))
				.splineToSplineHeading(new Pose2d(43, -10, Math.toRadians(180)), Math.toRadians(40))
				.strafeTo(new Vector2d(43, -53))

				.splineToSplineHeading(new Pose2d(45, -28, Math.toRadians(180)), Math.toRadians(100))
				.splineToSplineHeading(new Pose2d(52, -10, Math.toRadians(180)), Math.toRadians(40))
				.strafeTo(new Vector2d(52, -53))

				.splineToSplineHeading(new Pose2d(54, -28, Math.toRadians(180)), Math.toRadians(100))
				.splineToSplineHeading(new Pose2d(61, -10, Math.toRadians(180)), Math.toRadians(40))
				.strafeTo(new Vector2d(61, -53))

				.build();

			Action third = myBot.getDrive().actionBuilder(new Pose2d(61, -53, Math.toRadians(180)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();

			Action fourth = myBot.getDrive().actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
				.splineToLinearHeading(new Pose2d(9, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();

			Action fifth = myBot.getDrive().actionBuilder(new Pose2d(9, -35, Math.toRadians(270)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();

			Action sixth = myBot.getDrive().actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
				.splineToLinearHeading(new Pose2d(11, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();

			Action seventh = myBot.getDrive().actionBuilder(new Pose2d(11, -35, Math.toRadians(270)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();
				
			Action eighth = myBot.getDrive().actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
				.splineToLinearHeading(new Pose2d(13, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();
				
			Action park = myBot.getDrive().actionBuilder(new Pose2d(13, -35, Math.toRadians(270)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();

			myBot.runAction(new SequentialAction(
				first,
				second,
				third,
				fourth,
				fifth,
				sixth,
				seventh,
				eighth,
				park
			));
		} else if (OPTION == 2) {
			Action pushSamples = myBot.getDrive().actionBuilder(new Pose2d(24, -62, Math.toRadians(180)))
				.setReversed(true)
				.splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(270))
				.splineToConstantHeading(new Vector2d(43, -53), Math.toRadians(270))

				.splineToConstantHeading(new Vector2d(45, -28), Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(52, -10), Math.toRadians(270))
				.splineToConstantHeading(new Vector2d(52, -53), Math.toRadians(270))

				.splineToConstantHeading(new Vector2d(54, -28), Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(270))
				.splineToConstantHeading(new Vector2d(60, -53), Math.toRadians(270))
				.build();

			Action firstGrabSpecimen = myBot.getDrive().actionBuilder(new Pose2d(60, -53, Math.toRadians(180)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();
	

			Action grabSpecimen = myBot.getDrive().actionBuilder(new Pose2d(8, -35, Math.toRadians(270)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();
				
			Action hangSpecimen = myBot.getDrive().actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
				.splineToLinearHeading(new Pose2d(10, -35, Math.toRadians(270)), Math.toRadians(90))
				.build();

			Action pushSpecimen = myBot.getDrive().actionBuilder(new Pose2d(10, -35, Math.toRadians(270)))
				.strafeTo(new Vector2d(8, -35))
				.build();

			Action park = myBot.getDrive().actionBuilder(new Pose2d(8, -35, Math.toRadians(270)))
				.splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
				.build();

			myBot.runAction(new SequentialAction(
				pushSamples,
				firstGrabSpecimen,
				hangSpecimen,
				pushSpecimen,
				grabSpecimen,
				hangSpecimen,
				pushSpecimen,
				grabSpecimen,
				hangSpecimen,
				pushSpecimen,
				grabSpecimen,
				hangSpecimen,
				pushSpecimen,
				grabSpecimen,
				hangSpecimen,
				pushSpecimen,
				park
			));
		} else{
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