package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class RedAlliance implements Alliance {
	public String getColourString() {
		return "Red";
	}

	public Pose getGoalPose() {
		return new Pose(0, 144);
	}

	public double getHeadingOffset() {
		return 0;
	}

	public Pose getBasePose() {
		return new Pose(38, 34, Math.PI / 2);
	}
}
