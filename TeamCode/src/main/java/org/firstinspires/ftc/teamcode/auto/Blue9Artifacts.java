package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
@Autonomous(name = "Blue 9 Artifacts", group = "Autonomous")
public class Blue9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = true;

	private int pathState = 0;

	private Follower follower;
	private HardwareMap hardwareMap;
	private Pose startPose;

	private PathChain firstApproach, firstIntake, firstLaunch,
			secondIntake, secondLaunch;

	@Override
	public void runOpMode() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		waitForStart();

		while (opModeIsActive()) {
			pathUpdate();
		}
	}

	public void buildPaths() {
		firstApproach = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(96.000, 8.000),
								new Pose(93.061, 27.820),
								new Pose(104.000, 35.800)))
				.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(104.000, 35.800), new Pose(120.000, 35.800)))
				.setTangentHeadingInterpolation()
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(120.000, 35.800),
								new Pose(108.539, 35.853),
								new Pose(84.000, 76.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 76.000),
								new Pose(94.041, 58.776),
								new Pose(120.000, 59.800)))
				.setTangentHeadingInterpolation()
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(120.000, 59.800),
								new Pose(115.788, 59.755),
								new Pose(84.000, 76.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.build();
	}

	public void setPathState(int state) {
		pathState = state;
	}

	public void pathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy()) {
					follower.followPath(firstApproach, true);
					setPathState(1);
				}
				break;
			case 1:
				if (!follower.isBusy()) {
					follower.followPath(firstIntake, true);
					setPathState(2);
				}
				break;
			case 2:
				if (!follower.isBusy()) {
					follower.followPath(firstLaunch, true);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy()) {
					follower.followPath(secondIntake, true);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy()) {
					follower.followPath(secondLaunch, true);
					setPathState(-1);
				}
				break;
		}
	}
}
