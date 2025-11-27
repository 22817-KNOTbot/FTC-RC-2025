package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
	private Pose resetPose = new Pose();
	private boolean autoDrive;
	private Pose autoDriveTarget;

	public MecanumDrive(HardwareMap hardwareMap) {
		follower = Constants.createFollowerRoadRunner(hardwareMap);
	}

	public void initialize() {
		follower.startTeleopDrive();
	}

	public void setHeadingOffset(double headingOffset) {
		this.headingOffset = headingOffset;
	}

	public void move(float forward, float lateral, float rotation) {
		updateLocalizers();

		if (autoDrive)
			return;

		double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(rotation), 1);
		follower.setTeleOpDrive(
				forward / denominator,
				-lateral / denominator,
				-rotation / denominator,
				false,
				headingOffset);
	}

	public void updateLocalizers() {
		follower.update();
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

	public void setStartingPose(Pose pose) {
		follower.setStartingPose(pose);
	}

	public void setPose(Pose pose){
		follower.setPose(pose);
		// follower.update();
	}

	public Pose getPose() {
		return follower.getPose();
	}

	public Vector getVelocity() {
		return follower.getVelocity();
	}

	public void setResetPose(Pose pose) {
		resetPose = pose;
	}

	public void resetPose() {
		follower.setPose(resetPose);
		// follower.update();
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