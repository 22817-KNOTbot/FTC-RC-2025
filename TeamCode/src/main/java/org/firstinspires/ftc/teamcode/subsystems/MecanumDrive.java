package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class MecanumDrive {
	private Pose holdPose = new Pose();
	private boolean holdingPose;
	private Follower follower;
	private double headingOffset;

	public MecanumDrive(HardwareMap hardwareMap) {
		follower = Constants.createFollower(hardwareMap);
	}

	public void initialize() {
		follower.startTeleopDrive();
	}

	public void setHeadingOffset(double headingOffset) {
		this.headingOffset = headingOffset;
	}

	public void move(float forward, float lateral, float rotation) {
		follower.update();

		double relativeHeading = follower.getPose().getHeading() + headingOffset;

		double forwardRotated = forward * Math.sin(-relativeHeading) + lateral * Math.cos(-relativeHeading);
		double lateralRotated = forward * Math.cos(-relativeHeading) - lateral * Math.sin(-relativeHeading);

		double denominator = Math.max(Math.abs(forwardRotated) + Math.abs(lateralRotated) + Math.abs(rotation), 1);
		follower.setTeleOpDrive(
				forwardRotated / denominator,
				-lateralRotated / denominator,
				-rotation / denominator,
				true);
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