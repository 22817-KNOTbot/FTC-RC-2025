package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoadRunnerToPedroLocalizer implements Localizer {
	private ThreeDeadWheelLocalizer localizer;
	private double inPerTick;

	private Pose2d startPose = new Pose2d(0, 0, 0);
	private Pose2d velocity = new Pose2d(0, 0, 0);

	public RoadRunnerToPedroLocalizer(HardwareMap hardwareMap, MecanumDrive.Params driveParams, ThreeDeadWheelLocalizer.Params localizerParams) {
		this(hardwareMap, driveParams, localizerParams, new Pose2d(0, 0, 0));
	}

	public RoadRunnerToPedroLocalizer(HardwareMap hardwareMap, MecanumDrive.Params driveParams, ThreeDeadWheelLocalizer.Params localizerParams, Pose2d startingPose) {
		inPerTick = driveParams.inPerTick;

		localizer = new ThreeDeadWheelLocalizer(hardwareMap, driveParams.inPerTick, startingPose);
	}

	public Pose getPose() {
		return rrToPedroPose(startPose).plus(rrToPedroPose(localizer.getPose()));
	}

	public Pose getVelocity() {
		return rrToPedroPose(velocity);
	}

	public Vector getVelocityVector() {
		return rrToPedroPose(velocity).getAsVector();
	}

	public void setStartPose(Pose startPose) {
		this.startPose = pedroToRrPose(startPose);
	}

	public void setPose(Pose pose) {
		localizer.setPose(pedroToRrPose(pose.minus(rrToPedroPose(startPose))));
	}

	public void update() {
		PoseVelocity2d velocity = localizer.update();
		this.velocity = new Pose2d(velocity.linearVel, velocity.angVel);
	}

	public double getTotalHeading() {
		return localizer.getPose().heading.log();
	}

	public double getForwardMultiplier() {
		return inPerTick;
	}

	public double getLateralMultiplier() {
		return inPerTick;
	}

	public double getTurningMultiplier() {
		return inPerTick;
	}

	public void resetIMU() {}
	public double getIMUHeading() {
		return Double.NaN;
	}

	public boolean isNAN() {
		Pose2d pose = localizer.getPose();
		return Double.isNaN(pose.position.x) || Double.isNaN(pose.position.y) || Double.isNaN(pose.heading.log());
	}

	public static Pose rrToPedroPose(Pose2d rrPose) {
		return new Pose(rrPose.position.x, rrPose.position.y, rrPose.heading.log());
	}

	public static Pose2d pedroToRrPose(Pose pedroPose) {
		return new Pose2d(pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading());
	}
}
