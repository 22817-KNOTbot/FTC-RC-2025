package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class RedAlliance implements Alliance {
	public String getColourString() {
		return "Red";
	}

	public Pose getGoalPose() {
		return new Pose(0, 144);
	}
}
