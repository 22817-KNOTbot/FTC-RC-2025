package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class RedAlliance implements Alliance {
	public static double goalPoseX = 140;
	public static double goalPoseY = 132;
	public static double goalPoseHeading = 0;

	public String getColourString() {
		return "Red";
	}

	public Pose getGoalPose() {
		return new Pose(goalPoseX, goalPoseY, goalPoseHeading);
	}

	public double getHeadingOffset() {
		return 0;
	}

	public Pose getBasePose() {
		return new Pose(38, 34, Math.PI / 2);
	}

	public Pose getResetPose() {
		return new Pose(9, 9, 0);
	}
}
