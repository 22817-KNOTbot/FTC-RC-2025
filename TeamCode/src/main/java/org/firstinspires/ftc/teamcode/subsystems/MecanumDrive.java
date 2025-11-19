package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.teleop.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

public class MecanumDrive {
	private Follower follower;
	private double headingOffset;
	private Pose holdPose = new Pose();
	private boolean holdingPose;
	private boolean autoDrive;
	private Pose autoDriveTarget;
	private TeleOp teleop;

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
		if (autoDrive)
			return;

		double relativeHeading = follower.getPose().getHeading() + headingOffset;

		double forwardRotated = lateral * Math.sin(-relativeHeading) + forward * Math.cos(-relativeHeading);
		double lateralRotated = lateral * Math.cos(-relativeHeading) - forward * Math.sin(-relativeHeading);

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

	public void setAutoDrive(boolean enable) {
		if (enable && !autoDrive && autoDriveTarget != null) {
			follower.followPath(
					follower.pathBuilder()
							.addPath(new Path(new BezierLine(follower::getPose, autoDriveTarget)))
							.setHeadingInterpolation(
									HeadingInterpolator.linearFromPoint(follower::getHeading, autoDriveTarget.getHeading(), 1))
							.build());
		}
		if (!enable && autoDrive) {
			follower.breakFollowing();
			initialize();
		}
		autoDrive = enable;
	}

	public void setAutoDriveTarget(Pose target) {
		autoDriveTarget = target;
	}

	public void setPoseFromAuto(Pose pose){
		follower.setPose(pose);
	}

	public Pose getPose() {
		return follower.getPose();
	}
	public Vector getVelocity() {
		return follower.getVelocity();
	}

	public void resetPose() {
		follower.setPose(new Pose(0, 0, -headingOffset));
	}

	public Pose getHoldPose() {
		return holdPose;
	}

	public boolean isAutoDrive() {
		return autoDrive;
	}

	public Pose getAutoDriveTarget() {
		return autoDriveTarget;
	}
}