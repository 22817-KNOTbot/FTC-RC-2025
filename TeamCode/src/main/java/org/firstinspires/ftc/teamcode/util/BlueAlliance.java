package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class BlueAlliance implements Alliance {
	public static double goalPoseX = 5;
	public static double goalPoseY = 130;
	public static double goalPoseHeading = 0;

	public String getColourString() {
		return "Blue";
	}

	public Pose getGoalPose() {
		return new Pose(goalPoseX, goalPoseY, goalPoseHeading);
	}

	public Pose getGoalShooterPose() {
		return new Pose(0, 144, 0);
	}

	public Integer getGoalAprilTagId() {
		return 20;
	}

	public double getHeadingOffset() {
		return Math.PI;
	}

	public Pose getBasePose() {
		return new Pose(106, 34, Math.PI / 2);
	}
	
	public Pose getResetPose() {
		return new Pose(135, 9, Math.PI);
	}

	public int getObeliskSidePriority() {
		return 1;
	}
}
