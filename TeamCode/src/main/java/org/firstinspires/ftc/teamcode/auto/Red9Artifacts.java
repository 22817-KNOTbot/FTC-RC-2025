package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.scoring.Artifact;

import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;

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

	private Automations automationsHandler;
	private Alliance alliance;
	private HardwareMap hardwareMap;

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
				actionsDo();
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
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(39.771, 35.461), new Pose(24.000, 35.800)))
				.setTangentHeadingInterpolation()
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(24.000, 35.800),
								new Pose(35.461, 35.853),
								new Pose(60.000, 76.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(60.000, 76.000),
								new Pose(49.959, 58.776),
								new Pose(24.000, 59.800)))
				.setTangentHeadingInterpolation()
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(24.000, 59.800),
								new Pose(28.212, 59.755),
								new Pose(60.000, 76.000)))
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
					break;
				case 2:
					automationsHandler.intakeToggle();
					pattern = automationsHandler.getPatternShort().getPattern();
					automationsHandler.shootArtifact(pattern[0]);
					automationsHandler.shootArtifact(pattern[1]);
					automationsHandler.shootArtifact(pattern[2]);
					break;
				case 3:
					automationsHandler.intakeToggle();
					break;
				case 4:
					pattern = automationsHandler.getPatternShort().getPattern();
					automationsHandler.shootArtifact(pattern[0]);
					automationsHandler.shootArtifact(pattern[1]);
					automationsHandler.shootArtifact(pattern[2]);
					break;

			}
		}
	}
}
