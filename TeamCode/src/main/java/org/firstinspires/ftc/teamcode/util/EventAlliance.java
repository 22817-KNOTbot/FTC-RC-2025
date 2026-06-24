package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class EventAlliance implements Alliance {
	public static double goalPoseX = 16;
	public static double goalPoseY = 0;
	public static double goalPoseHeading = 0;

	public String getColourString() {
		return "Outreach Event";
	}

	public Pose getGoalPose() {
		return new Pose(goalPoseX, goalPoseY, goalPoseHeading);
	}

	public Pose getGoalShooterPose() {
		return new Pose(goalPoseX, goalPoseY, goalPoseHeading);
	}

	public Integer getGoalAprilTagId() {
		return null;
	}

	public double getHeadingOffset() {
		return 0;
	}

	public Pose getBasePose() {
		return new Pose();
	}
	
	public Pose getResetPose() {
		return new Pose();
	}

	public int getObeliskSidePriority() {
		return -1;
	}
}
