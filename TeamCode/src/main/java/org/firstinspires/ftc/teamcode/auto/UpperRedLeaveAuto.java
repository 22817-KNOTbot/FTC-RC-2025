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
@Autonomous(name = "Upper Red Leave", group = "Autonomous")
public class UpperRedLeaveAuto extends LinearOpMode {
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

	// blackboard
	public static final String POSE = "pose";
	public static final String ALLIANCE = "alliance";

	private PathChain leaveline;

	@Override
	public void runOpMode() {
		Object blackboardObject = blackboard.getOrDefault(POSE, new Pose(123.000, 124.000));
		blackboard.put(ALLIANCE, alliance);
		blackboard.put(POSE, new Pose(123.000, 124.000));

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
		leaveline = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(123.000, 124.000), new Pose(107.000, 130.000)))
				.setConstantHeadingInterpolation(Math.toRadians(215))
				.addParametricCallback(0, this::backboardPose)
				.build();
	}

	public void setPathState(int state) {
		pathState = state;
	}

	public void pathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy()) {
					follower.followPath(leaveline, true);
					setPathState(-1);
				}
		}
	}

	public void backboardPose() {
		blackboard.put(POSE, follower.getPose());
	}
}