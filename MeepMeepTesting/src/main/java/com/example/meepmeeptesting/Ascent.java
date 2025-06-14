package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Ascent {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(700);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
			// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
			.setConstraints(50, 50, Math.PI, Math.PI, 13)
			.build();
			
		TrajectoryActionBuilder park = myBot.getDrive().actionBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
			.strafeTo(new Vector2d(-36, -12))
			.turnTo(Math.toRadians(180))
			.strafeTo(new Vector2d(-20, -12));

		myBot.runAction(
			park.build()
		);

		meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}