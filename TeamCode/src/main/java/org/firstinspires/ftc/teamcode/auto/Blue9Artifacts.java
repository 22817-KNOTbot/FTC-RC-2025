package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;

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

	private Automations automationsHandler;
	private Alliance alliance;
	private ElapsedTime timer = new ElapsedTime();

	private int shootNumber = 0;

	private int pathState = 0;
	private int autoState = 0;
	private boolean DEBUG;

	private Follower follower;
	private Pose startPose;

	private Artifact.Colour[] pattern;

	private PathChain firstApproach, firstIntake, firstLaunch,
			secondIntake, secondLaunch;

	@Override
	public void runOpMode() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		automationsHandler = new Automations(hardwareMap, alliance, DEBUG);

		waitForStart();

		while (opModeIsActive()) {
			follower.update();
			if (doMovement) {
				pathUpdate();
				automationsHandler.updatePose(follower.getPose());
			}

			if (doActions) {
				automationsHandler.automationLoop();
				// actionsDo();
			}
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

	public void setActionState(int state) {
		autoState = state;
	}

	public void pathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy()) {
					actionsDo();
					follower.followPath(firstApproach, true);
					setActionState(1);
					setPathState(1);
				}
				break;
			case 1:
				if (!follower.isBusy()) {
					follower.followPath(firstIntake, true);
					actionsDo();
					setActionState(2);
					setPathState(2);
				}
				break;
			case 2:
				if (!follower.isBusy()) {
					follower.followPath(firstLaunch, true);
					actionsDo();
					setActionState(3);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy()) {
					follower.followPath(secondIntake, true);
					actionsDo();
					setActionState(4);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy()) {
					follower.followPath(secondLaunch, true);
					actionsDo();
					setActionState(-1);
					setPathState(-1);
				}
				break;
		}
	}

	public void actionsDo() {
		if (doActions) {
			switch (autoState) {
				case 0:
					break;
				case 1:
					automationsHandler.intakeToggle();
					timer.reset();
					shootNumber = 0;
					break;
				case 2:
					automationsHandler.intakeToggle();
					pattern = automationsHandler.getPatternShort().getPattern();
					if (timer.time() > 1 && timer.time() < 1.5 && shootNumber == 0) {
						automationsHandler.shootArtifact(pattern[0]);
						shootNumber = 1;
					} else if (timer.time() > 1.5 && timer.time() < 2 && shootNumber == 1) {
						automationsHandler.shootArtifact(pattern[1]);
						shootNumber = 2;
					} else if (timer.time() > 2 && shootNumber == 2) {
						automationsHandler.shootArtifact(pattern[2]);
						shootNumber = -1;
					}
					break;
				case 3:
					automationsHandler.intakeToggle();
					timer.reset();
					shootNumber = 0;
					break;
				case 4:
					automationsHandler.intakeToggle();
					pattern = automationsHandler.getPatternShort().getPattern();
					if (timer.time() > 1 && timer.time() < 1.5 && shootNumber == 0) {
						automationsHandler.shootArtifact(pattern[0]);
						shootNumber = 1;
					} else if (timer.time() > 1.5 && timer.time() < 2 && shootNumber == 1) {
						automationsHandler.shootArtifact(pattern[1]);
						shootNumber = 2;
					} else if (timer.time() > 2 && shootNumber == 2) {
						automationsHandler.shootArtifact(pattern[2]);
						shootNumber = -1;
					}
					break;

			}
		}
	}
}
