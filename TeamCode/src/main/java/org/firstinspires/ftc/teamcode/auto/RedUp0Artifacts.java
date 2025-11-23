package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;

import org.firstinspires.ftc.teamcode.subsystems.Storage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
@Autonomous(name = "Upper Red Leave", group = "Autonomous")
public class RedUp0Artifacts extends LinearOpMode {
	public static boolean doActions = true;
	public static boolean DEBUG = false;

	private boolean shooting = false;
	private int pathState = 0;
	private int shotsFired = 0;

	private Alliance alliance = new RedAlliance();
	private Follower follower;
	private Storage storage;
	private Pose startPose = follower.getPose();
	private Artifact.Colour[] patternColours;

	private PathChain leaveline;

	@Override
	public void runOpMode() {
		blackboard.put("alliance", alliance);
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();
		storage.resetArtifacts();

		waitForStart();

		while (opModeIsActive()) {
			follower.update();
			pathUpdate();
		}
		blackboard.put("pose", follower.getPose());
	}

	public void buildPaths() {
		leaveline = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(123.000, 124.000), new Pose(107.000, 130.000)))
				.setConstantHeadingInterpolation(Math.toRadians(215))
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
}