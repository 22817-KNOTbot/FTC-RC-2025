package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;


@Configurable
@Autonomous(name = "Red 9 Artifacts", group = "Autonomous")
public class Red9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = true;

	private int pathState = 0;
	private int actionState = 0;

	private Automations automationHandler;
	private Follower follower;
	private Pose startPose;
	private Artifact.Colour[] patternColours;

	private PathChain firstApproach, firstIntake, firstLaunch,
			secondIntake, secondLaunch;

	@Override
	public void runOpMode() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		waitForStart();

		while (opModeIsActive()) {
			// get motif patterns
			while (patternColours == null) {
				patternColours = automationHandler.getArtifactPattern().getPattern();
			}
			automationHandler.automationLoop();
			if (doActions) {
				automationHandler.updatePose(follower.getPose());
				automationHandler.updateTurret();
			}
			if (doMovement) {
				follower.update();
				pathUpdate();
			}
		}
	}

	public void buildPaths() {
		firstApproach = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(48.000, 8.000),
								new Pose(50.939, 27.820),
								new Pose(39.771, 35.461)))
				.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
				.addParametricCallback(0, this::readyToShoot)
				.addParametricCallback(0, this::shootArtifacts)
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(39.771, 35.461), new Pose(24.000, 35.800)))
				.setTangentHeadingInterpolation()
				.addParametricCallback(0, this::intakeToggle)
				.addParametricCallback(1, this::intakeToggle)
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(24.000, 35.800),
								new Pose(35.461, 35.853),
								new Pose(60.000, 76.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(1, this::shootArtifacts)
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(60.000, 76.000),
								new Pose(49.959, 58.776),
								new Pose(24.000, 59.800)))
				.setTangentHeadingInterpolation()
				.addParametricCallback(0.4, this::intakeToggle)
				.addParametricCallback(1, this::intakeToggle)
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(24.000, 59.800),
								new Pose(28.212, 59.755),
								new Pose(60.000, 76.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(1, this::shootArtifacts)
				.build();
	}

	public void setPathState(int state) {
		pathState = state;
	}

	public void setActionState(int state) {
		actionState = state;
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

	public void readyToShoot() {
		if (doActions) {
			follower.pausePathFollowing();
			patternColours = automationHandler.getArtifactPattern().getPattern();
		}
	}

	public void shootArtifacts() {
		if (doActions) {
			follower.pausePathFollowing();
			for (Artifact.Colour colour : patternColours) {
				automationHandler.prepareOrShootArtifact(colour);
			}
			follower.resumePathFollowing();
		}
	}

	public void intakeToggle() {
		if (doActions) {
			automationHandler.intakeToggle();
		}
	}
}
