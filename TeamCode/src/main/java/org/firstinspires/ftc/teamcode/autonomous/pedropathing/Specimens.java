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
	private Intake intake;

	private Follower follower;
	private Timer pathTimer, parametricActionTimer, actionTimer, opmodeTimer;
	private int pathState;
	private boolean parametricActionTimerReset, actionTimerReset;
	private boolean actionInit;
	private boolean actionDone;

private PathChain
		hangSpecimenPre, 
		grabSample1, releaseSample1,
		grabSample2, releaseSample2,
		grabSample3, releaseSample3,
		grabSpecimen1, hangSpecimen1,
		grabSpecimen2, hangSpecimen2,
		grabSpecimen3, hangSpecimen3,
		grabSpecimen4, hangSpecimen4,
		park;

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
			telemetry.addData("Follower Busy", follower.isBusy());
			telemetry.addData("Parametric End", follower.atParametricEnd());
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
		hangSpecimenPre = follower.pathBuilder()
			.addPath(
				// Line 1
				new BezierLine(
					new Point(startPose),
					new Point(37.400, 63.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSample1 = follower.pathBuilder()
			.addPath(
				// Line 2
				new BezierCurve(
					new Point(37.000, 63.000, Point.CARTESIAN),
					new Point(28.000, 65.500, Point.CARTESIAN),
					new Point(28.500, 43.500, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(308))
			.build();

		releaseSample1 = follower.pathBuilder()
			.addPath(
				// Line 3
				new BezierLine(
				new Point(28.500, 43.500, Point.CARTESIAN),
				new Point(28.000, 38.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(308), Math.toRadians(221))
			.build();

		grabSample2 = follower.pathBuilder()
			.addPath(
				// Line 4
				new BezierLine(
				new Point(28.000, 38.000, Point.CARTESIAN),
				new Point(28.000, 33.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(221), Math.toRadians(309))
			.build();

		releaseSample2 = follower.pathBuilder()
			.addPath(
				// Line 5
				new BezierLine(
				new Point(28.000, 33.000, Point.CARTESIAN),
				new Point(28.000, 27.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(309), Math.toRadians(221))
			.build();

		grabSample3 = follower.pathBuilder()
			.addPath(
				// Line 6
				new BezierLine(
				new Point(28.000, 27.000, Point.CARTESIAN),
				new Point(28.000, 23.500, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(221), Math.toRadians(309))
			.build();

		releaseSample3 = follower.pathBuilder()
			.addPath(
				// Line 7
				new BezierLine(
				new Point(28.500, 23.000, Point.CARTESIAN),
				new Point(18.000, 24.000, Point.CARTESIAN)
				)
			)
			.setLinearHeadingInterpolation(Math.toRadians(309), Math.toRadians(200))
			.build();

		grabSpecimen1 = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierCurve(
					new Point(18.000, 24.000, Point.CARTESIAN),
					new Point(20.000, 37.000, Point.CARTESIAN),
					new Point(9.500, 36.000, Point.CARTESIAN)
				  )
			)
			.setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
			.build();

		hangSpecimen1 = follower.pathBuilder()
			.addPath(
				// Line 6
				new BezierCurve(
					new Point(9.500, 36.000, Point.CARTESIAN),
					new Point(13.000, 36.000, Point.CARTESIAN),
					new Point(22.000, 68.522, Point.CARTESIAN),
					new Point(37.800, 68.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen2 = follower.pathBuilder()
			.addPath(
				// Line 7
				new BezierCurve(
					new Point(37.800, 68.000, Point.CARTESIAN),
					new Point(22.486, 68.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(10.300, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen2 = follower.pathBuilder()
			.addPath(
				// Line 8
				new BezierCurve(
					new Point(10.300, 36.000, Point.CARTESIAN),
					new Point(13.000, 36.000, Point.CARTESIAN),
					new Point(22.000, 70.522, Point.CARTESIAN),
					new Point(37.800, 70.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen3 = follower.pathBuilder()
			.addPath(
				// Line 9
				new BezierCurve(
					new Point(37.800, 70.000, Point.CARTESIAN),
					new Point(22.486, 68.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(10.300, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen3 = follower.pathBuilder()
			.addPath(
				// Line 10
				new BezierCurve(
					new Point(10.000, 36.000, Point.CARTESIAN),
					new Point(13.000, 36.000, Point.CARTESIAN),
					new Point(22.000, 72.522, Point.CARTESIAN),
					new Point(37.800, 72.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		grabSpecimen4 = follower.pathBuilder()
			.addPath(
				// Line 11
				new BezierCurve(
					new Point(37.800, 72.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(10.600, 36.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		hangSpecimen4 = follower.pathBuilder()
			.addPath(
				// Line 12
				new BezierCurve(
					new Point(10.600, 36.000, Point.CARTESIAN),
					new Point(13.000, 36.000, Point.CARTESIAN),
					new Point(22.000, 74.522, Point.CARTESIAN),
					new Point(37.800, 74.000, Point.CARTESIAN)
				)
			)
			.setConstantHeadingInterpolation(Math.toRadians(180))
			.build();

		park = follower.pathBuilder()
			.addPath(
				// Line 13
				new BezierCurve(
					new Point(37.800, 74.000, Point.CARTESIAN),
					new Point(22.486, 67.000, Point.CARTESIAN),
					new Point(23.000, 36.000, Point.CARTESIAN),
					new Point(9.500, 36.000, Point.CARTESIAN)
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
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSample1, true);
					setPathState(2);
				}
				break;
			case 2:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(releaseSample1, true);
					setPathState(3);
				}
				break;
			case 3:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSample2, true);
					setPathState(4);
				}
				break;
			case 4:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(releaseSample2, true);
					setPathState(5);
				}
				break;
			case 5:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSample3, true);
					setPathState(6);
				}
				break;
			case 6:
				if (!follower.isBusy() && actionDone) {
					follower.followPath(releaseSample3, true);
					setPathState(7);
				}
				break;
			case 7:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSpecimen1, true);
					setPathState(8);
				}
				break;
			case 8:
				if ((!follower.isBusy() || follower.atParametricEnd() /* follower.isRobotStuck() */) && actionDone) {
					follower.followPath(hangSpecimen1, true);
					setPathState(9);
				}
				break;
			case 9:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSpecimen2, true);
					setPathState(10);
				}
				break;
			case 10:
				if ((!follower.isBusy() || follower.atParametricEnd() /* follower.isRobotStuck() */) && actionDone) {
					follower.followPath(hangSpecimen2, true);
					setPathState(11);
				}
				break;
			case 11:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSpecimen3, true);
					setPathState(12);
				}
				break;
			case 12:
				if ((!follower.isBusy() || follower.atParametricEnd() /* follower.isRobotStuck() */) && actionDone) {
					follower.followPath(hangSpecimen3, true);
					setPathState(13);
				}
				break;
			case 13:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(grabSpecimen4, true);
					setPathState(14);
				}
				break;
			case 14:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(hangSpecimen4, true);
					setPathState(15);
				}
				break;
			case 15:
				if ((!follower.isBusy() || follower.atParametricEnd()) && actionDone) {
					follower.followPath(park, true);
					setPathState(16);
				}
				break;
			case 16:
				if (!follower.isBusy() && actionDone) {
					setPathState(-1);
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
					actionDone = grabSpecimenFirst(0);
				break;
			case 1:
				if ((!follower.isBusy() || follower.atParametricEnd())) {
					actionDone = hangSpecimenFirst();
					if (actionDone) {
						retractSlides();
					}
				}
				break;
			case 2:
				if (!actionInit) {
					actionInit = grabSampleInit(-49);
				}
			case 4:
				if (!follower.isBusy()) {
					actionDone = grabSample();
				}
				break;
			case 6:
				if (!follower.isBusy()) {
					if (!actionInit) {
						grabSampleInit(-49);
						if (parametricActionTimer.getElapsedTimeSeconds() < 0.7) break;
						actionInit = grabSample(0.5);
					} else {
						actionDone = retractIntake();
					}
				}
				break;
			case 3:
				if ((!follower.isBusy() || follower.atParametricEnd())) {
					releaseSample();
					actionDone = true;
				}
				break;
			case 5:
				if ((!follower.isBusy() || follower.atParametricEnd())) {
					if (!actionInit) {
						actionInit = true;
						releaseSample();
					} else {
						actionDone = retractIntake();
					}
				}
				break;
			case 7:
				if (!follower.isBusy() || follower.atParametricEnd()) {
					if (!actionInit) {
						actionInit = true;
						releaseSample();
					} else {
						actionDone = sampleToSpecimen();
					}
				}
				break;
			case 8:
			case 10:
			case 12:
			case 14:
				if (!actionInit && pathTimer.getElapsedTimeSeconds() > 0.7) {
					grabSpecimenInit();
					actionInit = true;
				} else if ((!follower.isBusy() || follower.atParametricEnd())) {
					actionDone = grabSpecimen();
				}
				break;
			case 9:
			case 11:
			case 13:
			case 15:
				if ((!follower.isBusy()/* || follower.atParametricEnd() */)) {
					actionDone = hangSpecimen();
				}
				break;
			case 16:
				actionDone = grabSpecimenInit();
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
		intake.setPosition(Intake.Positions.INTAKE);
		if (actionTimer.getElapsedTimeSeconds() < delay) return false;
		intake.closeClaw();
		if (actionTimer.getElapsedTimeSeconds() < delay+0.3) return false;
		intake.setPosition(Intake.Positions.PRE_INTAKE);

		return true;
	}

	public boolean retractSlides() {
		slides.setPosition(Slides.Positions.RETRACTED);

		return true;
	}

	public boolean retractIntake() {
		intake.setPosition(Intake.Positions.POST_INTAKE);

		return true;
	}

	public boolean releaseSample() {
		intake.openClaw();

		return actionTimer.getElapsedTimeSeconds() > 0.3;
	}

	public boolean sampleToSpecimen() {
		if (parametricActionTimer.getElapsedTimeSeconds() < 0.8) {
			intake.setPosition(Intake.Positions.RETRACTED);
		} else {
			cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
			return true;
		}

		return false;
	}

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
		return grabSpecimen(0.3);
	}

	public boolean grabSpecimenFirst(double delay) {
		claw.setPosition(Claw.Positions.CLOSED);
		if (parametricActionTimer.getElapsedTimeSeconds() < delay) return false;
		slides.setPosition(Slides.Positions.HIGH_CHAMBER_PREHANG);
		cv4b.setPosition(CV4B.SPECIMEN_HANG_DRIVE + CV4B.offset_drive + 0.02, CV4B.SPECIMEN_HANG_COAX + CV4B.offset_coax + 0.007);

		return true;
	}

	public boolean grabSpecimen(double delay) {
		claw.setPosition(Claw.Positions.CLOSED);
		if (parametricActionTimer.getElapsedTimeSeconds() < delay) return false;
		slides.setPosition(Slides.Positions.HIGH_CHAMBER_PREHANG);
		cv4b.setPosition(CV4B.Positions.SPECIMEN_HANG);

		return true;
	}

	public boolean hangSpecimenFirst() {
		if (parametricActionTimer.getElapsedTimeSeconds() < 0.8) {
			slides.setPosition(820);
			return false;
		} else {
			claw.setPosition(Claw.Positions.OPEN);
			return true;
		}
	}

	public boolean hangSpecimen() {
		if (actionTimer.getElapsedTimeSeconds() < 0.3) {
			slides.setPosition(800);
			return false;
		} else {
			claw.setPosition(Claw.Positions.OPEN);
			return true;
		}
	}
}