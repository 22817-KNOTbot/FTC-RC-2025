package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public interface Alliance {
	public String getColourString();
	public Pose getGoalPose();
	public Pose getGoalShooterPose();
	public double getHeadingOffset();
	public Pose getBasePose();
	public Pose getResetPose();
	public int getObeliskSidePriority();
}