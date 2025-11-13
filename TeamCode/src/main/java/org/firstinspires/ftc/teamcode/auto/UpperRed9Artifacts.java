package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;


@Configurable
@Autonomous(name = "Upper Red 9 Artifacts", group = "Autonomous")
public class UpperRed9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = true;
	public static boolean DEBUG = false;

	private boolean shooting = false;
	private int pathState = 0;
	private int shotsFired = 0;

	private Alliance alliance = new RedAlliance();
	private Automations automationHandler;
	private Follower follower;
	private Pose startPose;
	private Artifact.Colour[] patternColours;

	private PathChain firstApproach, firstIntake, firstLaunch,
			secondApproch, secondIntake, secondLaunch, exitShootingZone;

	@Override
	public void runOpMode() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		automationHandler = new Automations(hardwareMap, alliance, DEBUG);
		buildPaths();

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

		automationHandler.end();
	}

	public void buildPaths() {
		firstApproach = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 136.000),
								new Pose(90.000, 84.000),
								new Pose(104.000, 84.000)))
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
				.setReversed()
				.addParametricCallback(0, this::readyToShoot)
				.addParametricCallback(0, this::startShooting)
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(104.000, 84.000), new Pose(120.000, 84.000)))
				.setTangentHeadingInterpolation()
				.addParametricCallback(0, this::intakeToggle)
				.addParametricCallback(1, this::intakeToggle)
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(120.000, 84.000),
								new Pose(100.000, 84.000),
								new Pose(90.000, 96.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(1, this::startShooting)
				.build();
		
		secondApproch = follower.pathBuilder()
				.addPath(
					new BezierCurve(
			    			new Pose(90.000, 96.000),
			    			new Pose(102.000, 82.000),
			    			new Pose(90.000, 60.000),
			    			new Pose(105.000, 60.000)))
				.setTangentHeadingInterpolation()
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(105.000, 60.000), new Pose(120.000, 60.000)))
				.setTangentHeadingInterpolation()
				.addParametricCallback(0, this::intakeToggle)
				.addParametricCallback(1, this::intakeToggle)
				.build();

    	secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(120.000, 60.000),
								new Pose(100.000, 60.000),
								new Pose(84.000, 84.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(1, this::startShooting)
				.build();

		exitShootingZone = follower
					.pathBuilder()
					.addPath(
		    			new BezierLine(new Pose(84.000, 84.000), new Pose(95.000, 68.000)))
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
					follower.followPath(secondApproch, true);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy()) {
					follower.followPath(secondIntake, true);
					setPathState(4);
				}
				break;
			case 5:
				if (!follower.isBusy()) {
					follower.followPath(secondLaunch, true);
					setPathState(6);
				}
				break;
			case 6:
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
