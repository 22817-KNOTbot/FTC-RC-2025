package org.firstinspires.ftc.teamcode.autonomous.pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "5 Specimen Auto - Pedro", group = "Autonomous")
public class Specimens extends LinearOpMode {
	public static double startPoseX = 9;
	public static double startPoseY = 63;
	public static double startPoseHeading = 180;
	public static boolean doMovement = true;
	public static boolean doActions = true;
	private Pose startPose;
	private Slides slides;
	private CV4B cv4b;
	private Claw claw;

	private Follower follower;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private int pathState;
	private boolean actionInit;
	private boolean actionDone;

private PathChain
		hangSpecimenPre, 
		pushSamples, hangSpecimen1,
		grabSpecimen2, hangSpecimen2,
		grabSpecimen3, hangSpecimen3,
		grabSpecimen4, hangSpecimen4,
		park;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseHeading));

		pathTimer = new Timer();
		actionTimer = new Timer();
		opmodeTimer = new Timer();

		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		slides = new Slides(hardwareMap, true);
		cv4b = new CV4B(hardwareMap);
		claw = new Claw(hardwareMap);

		waitForStart();

		opmodeTimer.resetTimer();
		setPathState(0);

		while (opModeIsActive()) {
			follower.update();
			if (doActions) autonomousActionUpdate();
			if (doMovement) autonomousPathUpdate();

			telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
			telemetry.addData("Path State", pathState);
			telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
			telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
			telemetry.addData("Action Done", actionDone);
			telemetry.addData("x", follower.getPose().getX());
			telemetry.addData("y", follower.getPose().getY());
			telemetry.addData("heading", follower.getPose().getHeading());
			telemetry.update();
		}
	}

	public void buildPaths() {
		hangSpecimenPre = follower.pathBuilder()
			.addPath(
				// Line 1
				new BezierLine(
					new Point(startPose),
					new Point(39.000, 63.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		pushSamples = follower.pathBuilder()
			.addPath(
				// Line 2
				new BezierCurve(
					new Point(39.000, 63.000, Point.CARTESIAN),
					new Point(30.397, 37.610, Point.CARTESIAN),
					new Point(39.000, 35.292, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.addPath(
				// Line 3
				new BezierCurve(
					new Point(39.000, 35.292, Point.CARTESIAN),
					new Point(103.814, 31.428, Point.CARTESIAN),
					new Point(14.000, 23.957, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.addPath(
				// Line 4
				new BezierCurve(
					new Point(14.000, 23.957, Point.CARTESIAN),
					new Point(117.725, 22.154, Point.CARTESIAN),
					new Point(14.426, 15.199, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.addPath(
				// Line 5
				new BezierCurve(
					new Point(14.426, 15.199, Point.CARTESIAN),
					new Point(122.361, 8.243, Point.CARTESIAN),
					new Point(9.000, 9.016, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen1 = follower.pathBuilder()
			.addPath(
				// Line 6
				new BezierCurve(
					new Point(9.000, 9.016, Point.CARTESIAN),
					new Point(36.580, 36.000, Point.CARTESIAN),
					new Point(15.456, 68.522, Point.CARTESIAN),
					new Point(39.000, 67.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen2 = follower.pathBuilder()
			.addPath(
				// Line 7
				new BezierCurve(
					new Point(39.000, 67.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(9.000, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen2 = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierCurve(
					new Point(9.000, 36.000, Point.CARTESIAN),
					new Point(36.580, 36.000, Point.CARTESIAN),
					new Point(15.456, 69.522, Point.CARTESIAN),
					new Point(39.000, 69.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen3 = follower.pathBuilder()
			.addPath(
				// Line 9
				new BezierCurve(
					new Point(39.000, 69.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(9.000, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen3 = follower.pathBuilder()
			.addPath(
				// Line 10
				new BezierCurve(
					new Point(9.000, 36.000, Point.CARTESIAN),
					new Point(36.580, 36.000, Point.CARTESIAN),
					new Point(15.456, 71.522, Point.CARTESIAN),
					new Point(39.000, 71.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen4 = follower.pathBuilder()
			.addPath(
				// Line 11
				new BezierCurve(
					new Point(39.000, 71.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(9.000, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen4 = follower.pathBuilder()
			.addPath(
				// Line 12
				new BezierCurve(
					new Point(9.000, 36.000, Point.CARTESIAN),
					new Point(36.580, 36.000, Point.CARTESIAN),
					new Point(15.456, 73.522, Point.CARTESIAN),
					new Point(39.000, 73.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		park = follower.pathBuilder()
			.addPath(
				// Line 13
				new BezierCurve(
					new Point(39.000, 73.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(9.000, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();	
	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(hangSpecimenPre, true);
					setPathState(1);
				}
				break;
			case 1:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(pushSamples, true);
					setPathState(2);
				}
				break;
			case 2:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(hangSpecimen1, true);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(grabSpecimen2, true);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(hangSpecimen2, true);
					setPathState(5);
				}
				break;
			case 5:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(grabSpecimen3, true);
					setPathState(6);
				}
				break;
			case 6:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(hangSpecimen3, true);
					setPathState(7);
				}
				break;
			case 7:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(grabSpecimen4, true);
					setPathState(8);
				}
				break;
			case 8:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(hangSpecimen4, true);
					setPathState(9);
				}
				break;
			case 9:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(park, true);
					setPathState(10);
				}
				break;
			case 10:
				if (!follower.isBusy() && actionDone) {
					setPathState(-1);
				}
		}
	}

	public void autonomousActionUpdate() {
		switch (pathState) {
			case 0:
				if (!actionInit) {
					actionDone = grabSpecimen();
					actionInit = true;
				}
				break;
			case 1:
				if (!follower.isBusy()) {
					actionDone = hangSpecimen();
				}
				break;
			case 2:
			case 4:
			case 6:
			case 8:
				if (!actionInit) {
					grabSpecimenInit();
					actionInit = true;
				} else if (!follower.isBusy()) {
					actionDone = grabSpecimen();
				}
				break;
			case 3:
			case 5:
			case 7:
			case 9:
				if (!follower.isBusy()) {
					actionDone = hangSpecimen();
				}
				break;
			case 10:
				actionDone = grabSpecimenInit();
		}
	}

	public void setPathState(int pState) {
		pathState = pState;
		pathTimer.resetTimer();
		actionTimer.resetTimer();
		actionInit = false;
		actionDone = !doActions;
	}

	/*
	 * Autonomous actions
	 */

	public boolean grabSpecimenInit() {
		cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		claw.setPosition(Claw.Positions.OPEN);
		slides.setPosition(Slides.Positions.RETRACTED);
		if (!slides.isSlideBusy()) {
			return true;
		}
		return false;
	}

	public boolean grabSpecimen() {
		claw.setPosition(Claw.Positions.CLOSED);
		slides.setPosition(Slides.Positions.HIGH_CHAMBER_PREHANG);
		cv4b.setPosition(CV4B.Positions.SPECIMEN_HANG);

		return true;
	}

	public boolean hangSpecimen() {
		if (actionTimer.getElapsedTimeSeconds() < 0.5) {
			slides.setPosition(Slides.Positions.HIGH_CHAMBER_HANG);
			return false;
		} else {
			claw.setPosition(Claw.Positions.OPEN);
			return true;
		}
	}
}
