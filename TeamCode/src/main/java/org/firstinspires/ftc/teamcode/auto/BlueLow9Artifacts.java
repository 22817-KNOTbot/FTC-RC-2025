package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
@Autonomous(name = "Blue 9 Artifacts", group = "Autonomous")
public class BlueLow9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = true;
	public static boolean resetEncoder = true;
	public static boolean DEBUG = false;

	private boolean shooting = false;
	private int pathState = 0;
	private int shotsFired = 0;
	private Alliance alliance = new BlueAlliance();
	private Automations automationHandler;
	private Follower follower;
	private Pose startPose = new Pose(48.000, 8.000);
	private Artifact.Colour[] patternColours;

	private PathChain firstApproach, firstIntake, firstLaunch,
			secondIntake, secondLaunch, exitShootingZone;

	@Override
	public void runOpMode() {
		blackboard.put("alliance", alliance);
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		automationHandler = new Automations(hardwareMap, alliance, resetEncoder, DEBUG);
		buildPaths();
		automationHandler.resetArtifacts();

		waitForStart();

		while (opModeIsActive()) {
			if (doActions) {
				automationHandler.automationLoop();
				automationHandler.updatePose(follower.getPose());
				automationHandler.updateTurret();
				if (shotsFired >= 3) {
					shotsFired = 0;
					shooting = false;
					follower.resumePathFollowing();
				}
				if (shooting) {
					if (automationHandler.getStorageState() == Automations.StorageState.WAITING) {
						automationHandler.prepareOrShootArtifact(patternColours[shotsFired]);
						shotsFired += 1;
					}
				}
			}
			if (doMovement) {
				follower.update();
				pathUpdate();
			}
		}
		blackboard.put("pose", follower.getPose());
		automationHandler.end();
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
				.addParametricCallback(0, this::startShooting)
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
				.addParametricCallback(1, this::startShooting)
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(60.000, 76.000),
								new Pose(49.959, 58.776),
								new Pose(24.000, 59.800)))
				.setTangentHeadingInterpolation()
				.addPoseCallback(new Pose(45.000, 63.000), this::intakeToggle, 0.4)
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
				.addParametricCallback(1, this::startShooting)
				.build();
		exitShootingZone = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(60.000, 76.000), new Pose(49.000, 70.000)))
				.setTangentHeadingInterpolation()
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
					setPathState(5);
				}
				break;
			case 5:
				if (!follower.isBusy()) {
					follower.followPath(exitShootingZone, true);
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

	public void startShooting() {
		if (doActions) {
			follower.pausePathFollowing();
			shooting = true;
		}
	}

	public void intakeToggle() {
		if (doActions) {
			automationHandler.intakeToggle();
		}
	}
}
