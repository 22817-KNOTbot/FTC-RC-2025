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

import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.ArrayList;

@Config
@Autonomous(name = "5 Sample Auto - Pedro", group = "Autonomous")
public class Samples5 extends LinearOpMode {
	public static double startPoseX = 9;
	public static double startPoseY = 105;
	public static double startPoseHeading = 270;
	public static boolean doMovement = true;
	public static boolean doActions = true;
	private Pose startPose;
	private Slides slides;
	private CV4B cv4b;
	private Claw claw;
	private Intake intake;

	private Follower follower;
	private Timer pathTimer, parametricActionTimer, actionTimer, opmodeTimer;
	private int pathState;
	private boolean parametricActionTimerReset, actionTimerReset;
	private boolean actionInit;
	private boolean actionDone;

	private ArrayList<Pose> extraSamplePoses = new ArrayList<>();
	private ArrayList<Integer> extraSampleRotations = new ArrayList<>();

	private PathChain
		firstSample,
		intakeFirst, depositFirst,
		intakeSecond, depositSecond,
		intakeThird, depositThird,
		intakeFourth, depositFourth,
		prePark, park;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseHeading));

		pathTimer = new Timer();
		parametricActionTimer = new Timer();
		actionTimer = new Timer();
		opmodeTimer = new Timer();

		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);

		slides = new Slides(hardwareMap, true);
		intake = new Intake(hardwareMap, true);
		cv4b = new CV4B(hardwareMap);
		claw = new Claw(hardwareMap);

		while (opModeInInit()) {
			if (gamepad1.touchpad_finger_1) {
				double x = (gamepad1.touchpad_finger_1_x * 22.25) + 72;
				double y = (gamepad1.touchpad_finger_1_y * 13.25) + 101;

				int rotation;
				double rotatedX = gamepad1.left_stick_x;
				double rotatedY = gamepad1.left_stick_y;
				
				if (rotatedX == 0 && rotatedY == 0) {
					continue;
				} else if (rotatedX == 0) {
					rotation = 0;
				} else if (rotatedY == 0) {
					rotation = 90;
				} else {
					double clippedAngle = ((Math.toDegrees(Math.atan2(rotatedX * (rotatedY / Math.abs(rotatedY)), Math.abs(rotatedY))) + 180) % 360) - 180;
					double roundedAngle = Math.round(clippedAngle / 5) * 5;
					rotation = (int) Math.round(roundedAngle);
				}

				if (gamepad1.touchpad) {
	
					extraSamplePoses.add(new Pose(x, y, 270));
	
					extraSampleRotations.add(rotation);
	
					gamepad1.rumble(1, 1, 100);
	
					telemetry.addLine("Submersible sample position saved!");
					break;
				}
				telemetry.addData("Sub Robot x", x);
				telemetry.addData("Sub Robot y", y);
				telemetry.addData("Sub Sample x", x);
				telemetry.addData("Sub Sample y", y - 29);
				telemetry.addData("Sub rot", rotation);
				telemetry.update();
			}
		}

		buildPaths();

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
			telemetry.addData("Follower Busy", follower.isBusy());
			telemetry.addData("Parametric Action Timer", parametricActionTimer.getElapsedTimeSeconds());
			telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
			telemetry.addData("Action Init", actionInit);
			telemetry.addData("Action Done", actionDone);
			telemetry.addData("x", follower.getPose().getX());
			telemetry.addData("y", follower.getPose().getY());
			telemetry.addData("heading", follower.getPose().getHeading());
			telemetry.update();
		}

		OpModeStorage.x = follower.getPose().getX();
		OpModeStorage.y = follower.getPose().getY();
		OpModeStorage.heading = follower.getPose().getHeading();
		OpModeStorage.mode = Automations.Modes.SPECIMEN;
	}

	public void buildPaths() {
		
		firstSample = follower.pathBuilder()
			.addPath(
				// Line 1
				new BezierCurve(
					new Point(9.000, 105.000, Point.CARTESIAN),
					new Point(19.500, 105.000, Point.CARTESIAN),
					new Point(17.000, 127.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
		.build();
		
		intakeFirst = follower.pathBuilder()
			.addPath(
				// Line 2
				new BezierLine(
					new Point(17.000, 127.000, Point.CARTESIAN),
					new Point(18.000, 123.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
		.build();
		
		depositFirst = follower.pathBuilder()
			.addPath(
				// Line 3
				new BezierLine(
					new Point(18.000, 123.000, Point.CARTESIAN),
					new Point(17.000, 127.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
		.build();
		
		intakeSecond = follower.pathBuilder()
			.addPath(
				// Line 4
				new BezierLine(
					new Point(17.000, 127.000, Point.CARTESIAN),
					new Point(23.000, 116.500, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(37))
		.build();
		
		depositSecond = follower.pathBuilder()
			.addPath(
				// Line 5
				new BezierLine(
					new Point(23.000, 116.500, Point.CARTESIAN),
					new Point(17.000, 127.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(315))
		.build();
		
		intakeThird = follower.pathBuilder()
			.addPath(
				// Line 6
				new BezierLine(
					new Point(17.000, 127.000, Point.CARTESIAN),
          			new Point(28.000, 121.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(51))
		.build();
		
		depositThird = follower.pathBuilder()
			.addPath(
				// Line 7
				new BezierLine(
					new Point(28.000, 121.000, Point.CARTESIAN),
         	 		new Point(17.000, 127.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(315))
		.build();
		
		prePark = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierCurve(
					new Point(17.000, 127.000, Point.CARTESIAN),
					new Point(75.000, 110.000, Point.CARTESIAN),
					new Point(72.000, 99.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
		.build();

		park = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierLine(
					new Point(72.000, 99.000, Point.CARTESIAN),
					new Point(72.000, 95.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(90))
		.build();

		/*
		 * Dynamic extra samples
		 */

		Pose fourthSamplePose = extraSamplePoses.get(0);

		intakeFourth = follower.pathBuilder()
			.addPath(
				new BezierCurve(
					new Point(17.000, 127.000),
					new Point(fourthSamplePose.getX(), fourthSamplePose.getY() + 30),
					new Point(fourthSamplePose)
				)
			)
			.setTangentHeadingInterpolation()
		.build();

		depositFourth = follower.pathBuilder()
			.addPath(
				new BezierCurve(
					new Point(fourthSamplePose),
					new Point(fourthSamplePose.getX(), 110),
					new Point(24, 120),
					new Point(17.000, 127.000)
				)
			)
			.setTangentHeadingInterpolation()
			.setReversed(true)
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
					follower.followPath(intakeFourth, true);
					setPathState(8);
				}
				break;
			case 8:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(depositFourth, true);
					setPathState(9);
				}
			case 9:
				if (!follower.isBusy() && actionDone) {
					// follower.followPath(prePark, true);
					setPathState(-1);
				}
				break;
			case 10:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(park);
					setPathState(11);
				}
				break;
			case 11:
				if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.5) {
					follower.breakFollowing();
					setPathState(11);
				}
			break;
		}
	}

	public void autonomousActionUpdate() {
		if (!follower.isBusy() && !actionTimerReset) {
			actionTimer.resetTimer();
			actionTimerReset = true;
		}

		if (follower.isBusy() && follower.atParametricEnd() && !parametricActionTimerReset) {
			parametricActionTimer.resetTimer();
			parametricActionTimerReset = true;
		}

		switch (pathState) {
			case 0:
				actionDone = true;
				break;
			case 1:
				if (!actionInit) {
					actionInit = depositInit();
				} else if (!follower.isBusy()) {
					actionDone = deposit();
				}
				break;
			case 2:
				if (!actionInit) {
					depositRetract();
					actionInit = grabSampleInit(0);
				} else if (!follower.isBusy()) {
					if (!grabSample()) break;
					actionDone = transferInit();
				}
				break;
			case 4:
				if (!actionInit) {
					depositRetract();
					actionInit = grabSampleInit(30);
				} else if (!follower.isBusy()) {
					if (!grabSample()) break;
					actionDone = transferInit();
				}
				break;
			case 6:
				if (!actionInit) {
					depositRetract();
					actionInit = grabSampleInit(51);
				} else if (!follower.isBusy()) {
					if (!grabSample()) break;
					actionDone = transferInit();
				}
				break;
			case 8:
				if (follower.isBusy() || !actionInit) {
					if (Math.abs(Math.toDegrees(follower.getPose().getHeading()) - 270) > 1) {
						depositRetract();
					} else {
						actionInit = grabSampleInit(extraSampleRotations.get(0));
					}
				} else if (actionInit) {
					if (!grabSample(0.5)) break;
					actionDone = transferInit();
				}
				break;
			case 3:
			case 5:
			case 7:
			case 9:
				if (!actionInit) {
					if (!transfer()) break;
					actionInit = depositInit(2);
				} else if (!follower.isBusy()) {
					actionDone = deposit();
				}
				break;
			case 10:
				if (!actionDone) {
					actionDone = ascentInit();
				}
				break;
			case 11:
				if (!actionDone) {
					actionDone = ascent();
				}
				break;
		}
		
		if (!doMovement && actionDone) {
			setPathState(pathState + 1);
		}
	}

	public void setPathState(int pState) {
		pathState = pState;
		pathTimer.resetTimer();
		actionTimer.resetTimer();
		actionTimerReset = false;
		parametricActionTimer.resetTimer();
		parametricActionTimerReset = false;
		actionInit = false;
		actionDone = !doActions;
	}

	/*
	 * Autonomous actions
	 */
	public boolean grabSampleInit(int angleDeg) {
		intake.setPosition(Intake.Positions.PRE_INTAKE);
		intake.setWristRotation(angleDeg * Intake.WRIST_VALUE_PER_DEG);

		return true;
	}

	public boolean grabSample() {
		return grabSample(0.3);
	}

	public boolean grabSample(double delay) {
		if (actionTimer.getElapsedTimeSeconds() < delay+0.5) {
			intake.setPosition(Intake.Positions.INTAKE);
			if (actionTimer.getElapsedTimeSeconds() < delay) return false;
			intake.closeClaw();
			if (actionTimer.getElapsedTimeSeconds() < delay+0.3) return false;
			intake.setPosition(Intake.Positions.PRE_INTAKE);
		}

		return true;
	}

	public boolean transferInit() {
		intake.setPosition(Intake.Positions.TRANSFER);
		intake.setSlidePosition(-100);
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		claw.setPosition(Claw.Positions.OPEN);

		if (intake.getSlidePosition() < 5) {
			return true;
		}
		return false;
	}

	public boolean transfer() {
		if (pathTimer.getElapsedTimeSeconds() < 0.5) {
			claw.setPosition(Claw.Positions.CLOSED);
		} else {
			intake.openClaw();
			return true;
		}
		return false;
	}

	public boolean depositInit() {
		return depositInit(1);
	}

	public boolean depositInit(double delay) {
		if (pathTimer.getElapsedTimeSeconds() < delay) {
			slides.setPosition(Slides.Positions.HIGH_BASKET);
		} else if (pathTimer.getElapsedTimeSeconds() < delay + 0.5) {
			cv4b.setPosition(CV4B.Positions.DEPOSIT);
		} else if (pathTimer.getElapsedTimeSeconds() > delay + 1.3) {
			return true;
		}
		return false;
	}

	public boolean deposit() {
		claw.setPosition(Claw.Positions.OPEN);
		return actionTimer.getElapsedTimeSeconds() > 0.3;
	}

	public boolean depositRetract() {
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		slides.setPosition(Slides.Positions.RETRACTED);

		return true;
	}

	public boolean ascentInit() {
		slides.setPosition(Slides.Positions.RETRACTED);
		if (actionTimer.getElapsedTimeSeconds() < 0.5) {
			cv4b.setPosition(CV4B.Positions.TRANSFER);
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
