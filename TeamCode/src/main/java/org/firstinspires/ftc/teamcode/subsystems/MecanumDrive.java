package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class MecanumDrive {
	private Pose holdPose = new Pose();
	private boolean holdingPose;
	private Follower follower;

	public MecanumDrive(HardwareMap hardwareMap) {
		follower = Constants.createFollower(hardwareMap);
	}

	public void initialize() {
		follower.startTeleopDrive();
	}

	public void move(float forward, float lateral, float rotation) {
		follower.update();
		double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(rotation), 1);
		follower.setTeleOpDrive(
			forward / denominator,
			-lateral / denominator,
			-rotation / denominator,
			false);
	}

	public void lockingMecanum(boolean enabled) {
		if (enabled && !holdingPose) {
			holdPose = follower.getPose();
			follower.holdPoint(holdPose);
			holdingPose = true;
		} else if (!enabled && holdingPose) {
			follower.breakFollowing();
			follower.startTeleopDrive();
			holdingPose = false;
		}
	}

	public Pose getPose() {
		return follower.getPose();
	}

	public void resetPose() {
		follower.setPose(new Pose());
	}

	public Pose getHoldPose() {
		return holdPose;
	}
}