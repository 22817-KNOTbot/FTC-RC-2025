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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "4 Sample Auto - Pedro", group = "Autonomous")
public class Samples extends LinearOpMode {
	public static double startPoseX = 9;
	public static double startPoseY = 63;
	public static double startPoseHeading = 180;
	public static boolean doMovement = true;
	public static boolean doActions = true;
	private Pose startPose;
	private Slides slides;
	private CV4B cv4b;
	private Claw claw;
	private Intake intake;

	private Follower follower;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private int pathState;
	private boolean actionInit;
	private boolean actionDone;

private PathChain
		firstSample,
		intakeFirst, depositFirst,
		intakeSecond, depositSecond,
		intakeThird, depositThird,
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
		intake = new Intake(hardwareMap, true);
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
		
		firstSample = follower.pathBuilder()
			.addPath(
				// Line 1
				new BezierCurve(
					new Point(9.000, 105.000, Point.CARTESIAN),
					new Point(19.500, 105.000, Point.CARTESIAN),
					new Point(16.000, 128.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
		.build();
		
		intakeFirst = follower.pathBuilder()
			.addPath(
				// Line 2
				new BezierLine(
					new Point(16.000, 128.000, Point.CARTESIAN),
					new Point(23.000, 120.500, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
		.build();
		
		depositFirst = follower.pathBuilder()
			.addPath(
				// Line 3
				new BezierLine(
					new Point(23.000, 120.500, Point.CARTESIAN),
					new Point(16.000, 128.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
		.build();
		
		intakeSecond = follower.pathBuilder()
			.addPath(
				// Line 4
				new BezierLine(
					new Point(16.000, 128.000, Point.CARTESIAN),
					new Point(23.000, 130.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
		.build();
		
		depositSecond = follower.pathBuilder()
			.addPath(
				// Line 5
				new BezierLine(
					new Point(23.000, 130.000, Point.CARTESIAN),
					new Point(16.000, 128.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
		.build();
		
		intakeThird = follower.pathBuilder()
			.addPath(
				// Line 6
				new BezierLine(
					new Point(16.000, 128.000, Point.CARTESIAN),
					new Point(34.000, 124.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(50))
		.build();
		
		depositThird = follower.pathBuilder()
			.addPath(
				// Line 7
				new BezierLine(
					new Point(34.000, 124.000, Point.CARTESIAN),
					new Point(16.000, 128.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(315))
		.build();
		
		park = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierCurve(
					new Point(16.000, 128.000, Point.CARTESIAN),
					new Point(75.000, 110.000, Point.CARTESIAN),
					new Point(72.000, 96.500, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
		.build();
	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(firstSample, true);
					setPathState(1);
				}
				break;
			case 1:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(intakeFirst, true);
					setPathState(2);
				}
				break;
			case 2:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(depositFirst, true);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(intakeSecond, true);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(depositSecond, true);
					setPathState(5);
				}
				break;
			case 5:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(intakeThird, true);
					setPathState(6);
				}
				break;
			case 6:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(depositThird, true);
					setPathState(7);
				}
				break;
			case 7:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(park, true);
					setPathState(8);
				}
				break;
			case 8:
				if (!follower.isBusy() && actionDone) {
					setPathState(-1);
				}
		}
	}

	public void autonomousActionUpdate() {
		switch (pathState) {
			case 0:
				depositInit();
				actionDone = true;
				break;
			case 1:
				if (!follower.isBusy()) {
					actionDone = deposit();
				}
				break;
			case 2:
			case 4:
			case 6:
				if (!actionInit) {
					actionInit = depositRetract();
				} else if (!follower.isBusy() && intake(Intake.SLIDE_POSITION_MAX, 0)) {
					actionDone = intakeRetract();
				}
				break;
			case 3:
			case 5:
			case 7:
				if (!actionInit) {
					actionInit = transfer();
				} else if (!follower.isBusy() && depositInit()) {
					actionDone = deposit();
				}
				break;
			case 8:
				if (!actionInit) {
					actionInit = ascentInit();
				} else if (!follower.isBusy()) {
					actionDone = ascent();
				}
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
	public boolean intake() {
		return intake(Intake.SLIDE_POSITION_DEFAULT, 0.5);
	}

	public boolean intake(int position) {
		return intake(position, 0.5);
	}

	public boolean intake(int position, double delay) {
		intake.setSlidePosition(position);
		intake.setPower(0.6);
		if (actionTimer.getElapsedTimeSeconds() > delay) {
			intake.setBucketPosition(Intake.Positions.INTAKE);
			if ((!intake.colourSensorResponding() || intake.getDistance(DistanceUnit.MM) <= 20)
				|| intake.isTouched()
				|| !intake.isSlideBusy()) {
					return true;
			}
		}
		return false;
	}

	public boolean intakeRetract() {
		intake.setPosition(Intake.Positions.TRANSFER);
		cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		claw.setPosition(Claw.Positions.OPEN);

		if (!intake.isSlideBusy()) {
			return true;
		}
		return false;
	}

	public boolean transfer() {
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		if (actionTimer.getElapsedTimeSeconds() > 0.5) {
			if (actionTimer.getElapsedTimeSeconds() > 1.1) {
				return true;
			} else if (actionTimer.getElapsedTimeSeconds() > 0.8) {
				intake.setPosition(Intake.Positions.POST_TRANSFER);
			} else {
				claw.setPosition(Claw.Positions.CLOSED);
				intake.setPower(0);
			}
		}
		return false;
	}

	public boolean depositInit() {
		if (!actionInit) {
			slides.setPosition(Slides.Positions.HIGH_BASKET);
			actionInit = true;
		} else if (actionTimer.getElapsedTimeSeconds() > 0.7) {
			if (actionTimer.getElapsedTimeSeconds() < 1.7) {
				cv4b.setPosition(CV4B.Positions.DEPOSIT);
				intake.setPower(-0.5);
			} else {
				intake.setPower(0);
				return true;
			}
		}
		return false;
	}

	public boolean deposit() {
		claw.setPosition(Claw.Positions.OPEN);
		return true;
	}

	public boolean depositRetract() {
		cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		slides.setPosition(Slides.Positions.RETRACTED);

		return true;
	}

	public boolean ascentInit() {
		slides.setPosition(Slides.Positions.RETRACTED);
		if (actionTimer.getElapsedTimeSeconds() < 0.5) {
			cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		} else {
			cv4b.setPosition(CV4B.Positions.DEPOSIT);
			return true;
		}
		return false;
	}

	public boolean ascent() {
		cv4b.setPosition(CV4B.Positions.LEVEL_ONE_ASCENT);
		claw.setPosition(Claw.Positions.CLOSED);

		return true;
	}
}
