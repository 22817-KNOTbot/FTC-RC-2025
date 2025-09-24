package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class BlueAlliance implements Alliance {
	public String getColourString() {
		return "Blue";
	}

	public Pose getGoalPose() {
		return new Pose(144, 144);
	}

	public double getHeadingOffset() {
		return -Math.PI;
	}

	public Pose getBasePose() {
		return new Pose(106, 34, Math.PI / 2);
	}
}
