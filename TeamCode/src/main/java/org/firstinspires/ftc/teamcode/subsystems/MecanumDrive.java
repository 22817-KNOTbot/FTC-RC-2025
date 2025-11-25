package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

public class MecanumDrive {
	private Follower follower;
	private Localizer rrLocalizer;
	private double headingOffset;
	private Pose holdPose = new Pose();
	private boolean holdingPose;
	private Pose resetPose = new Pose();
	private boolean autoDrive;
	private Pose autoDriveTarget;

	public MecanumDrive(HardwareMap hardwareMap) {
		follower = Constants.createFollower(hardwareMap);
		rrLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, Constants.localizerConstants.forwardTicksToInches, new Pose2d(0, 0, 0));
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

		rrLocalizer.update();
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

	public void setPose(Pose pose){
		follower.setPose(pose);
	}

	public Pose getPose() {
		return follower.getPose();
	}

	public Pose getRrPose() {
		Pose2d rrPose = rrLocalizer.getPose();
		return new Pose(rrPose.position.x, rrPose.position.y, rrPose.heading.log(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
	}

	public Vector getVelocity() {
		return follower.getVelocity();
	}

	public void setResetPose(Pose pose) {
		resetPose = pose;
	}

	public void resetPose() {
		follower.setPose(resetPose);
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