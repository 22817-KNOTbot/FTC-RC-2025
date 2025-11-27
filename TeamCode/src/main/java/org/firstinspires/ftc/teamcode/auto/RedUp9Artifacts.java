package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Config
@Configurable
@Autonomous(name = "Upper Red 9 Artifacts", group = "Autonomous")
public class RedUp9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = false;
	public static boolean DEBUG = false;
	public static double shooterVelocityTimeout = 5;

	private boolean shooting = false;
	private int pathState = 0;

	private Alliance alliance = new RedAlliance();
	private Automations automationHandler;
	private Follower follower;
	private Pose startPose = new Pose(123.000, 124.000, Math.toRadians(125));
	private Artifact.Colour[] patternColours;
	private ElapsedTime shootingTimer = new ElapsedTime();
	private boolean shootingTimerReset = false;

	private PathChain preloadLaunch, firstIntake, firstLaunch,
			secondApproach, secondIntake, secondLaunch, exitShootingZone;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
		
		blackboard.put("alliance", alliance);
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		follower.setPose(startPose);
		automationHandler = new Automations(hardwareMap, alliance, DEBUG);
		buildPaths();

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		while (opModeInInit()) {
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			automationHandler.updateMotifPattern();
		}

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			if (doActions) {
				automationHandler.updatePose(follower.getPose());
				automationHandler.updateVelocity(follower.getVelocity());
				automationHandler.updateTurret();
				automationHandler.updateShooter();

				if (shooting && shootingTimerReset && shootingTimer.time() >= shooterVelocityTimeout) {
					automationHandler.setIgnoreVelocity(true);
				}
				automationHandler.automationLoop();
				automationHandler.setIgnoreVelocity(false);
				if (shooting) {
					if (automationHandler.getStorageState() == Automations.StorageState.WAITING) {
						shooting = false;
						shootingTimerReset = false;
						follower.resumePathFollowing();
					} else if (automationHandler.getTransferState() == Storage.TransferState.RAMP_OUT) {
						if (!shootingTimerReset) {
							shootingTimer.reset();
							shootingTimerReset = true;
						}
					}
				}
			}
			if (doMovement) {
				follower.update();
				pathUpdate();
			}

			telemetryManager.addData("Pose", follower.getPose());

			if (DEBUG) {
				Drawing.drawRobot(follower.getPose(), telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();

				String[] debugLines = null;
				try {
					debugLines = follower.debug();
				} catch (Exception e) {
					telemetryManager.addLine("Failed to retrieve debug string!");
				}
				if (debugLines != null) {
					for (String line : debugLines) {
						telemetryManager.addLine(line);
					}
				}
			}
			telemetryManager.update();
		}
		blackboard.put("pose", follower.getPose());
		automationHandler.end();
	}

	public void buildPaths() {
		preloadLaunch = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(123.000, 124.000), new Pose(84.000, 84.000)))
				.setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
				// .addParametricCallback(1, this::readyToShoot)
				// .addParametricCallback(1, this::startShooting)
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(84.000, 84.000), new Pose(124.000, 83.000)))
				.setTangentHeadingInterpolation()
				// .addParametricCallback(0, this::intakeEnable)
				// .addParametricCallback(1, this::intakeDisable)
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(124.000, 83.000),
								new Pose(100.000, 84.000),
								new Pose(84.000, 84.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				// .addParametricCallback(1, this::startShooting)
				.build();

		secondApproach = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 84.000),
								new Pose(105.000, 84.000),
								new Pose(90.000, 60.000),
								new Pose(102.000, 60.000)))
				// .setTangentHeadingInterpolation()
				.setConstantHeadingInterpolation(0)
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(102.000, 60.000),
								new Pose(120.000, 60.000),
								new Pose(128.300, 55.000)))
				.setConstantHeadingInterpolation(Math.toRadians(0))
				// .addParametricCallback(0, this::intakeEnable)
				// .addParametricCallback(1, this::intakeDisable)
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(128.300, 55.000),
								new Pose(120.000, 55.000),
								new Pose(84.000, 84.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				// .addParametricCallback(1, this::startShooting)
				.build();

		exitShootingZone = follower
				.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 84.000),
								new Pose(94.000, 76.000),
								new Pose(90.000, 34.000),
								new Pose(102.000, 34.000)))
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
					follower.followPath(preloadLaunch, true);
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
				if (!follower.isBusy() || follower.isRobotStuck()) {
					follower.followPath(firstLaunch, true);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy()) {
					follower.followPath(secondApproach, true);
					setPathState(4);
				}
				break;
			case 4:
				if (follower.atParametricEnd() || follower.isRobotStuck()) {
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
			// case 7:
			// 	if (!follower.isBusy()) {
			// 		follower.followPath(null, true);
			// 		setPathState(-1);
			// 	}
			// 	break;
		}
	}

	public void readyToShoot() {
		if (doActions) {
			patternColours = automationHandler.getArtifactPattern().getPattern();
			automationHandler.setRapidFire(true);
		}
	}

	public void startShooting() {
		if (doActions) {
			follower.pausePathFollowing();
			if (automationHandler.prepareOrShootArtifactSequence(patternColours) == Storage.TurnDirection.NONE) {
				automationHandler.prepareOrShootAnyArtifact();
			};
			shooting = true;
		}
	}

	public void intakeEnable() {
		if (doActions) {
			automationHandler.intakeEnableActions(true);
		}
	}

	public void intakeDisable() {
		if (doActions) {
			automationHandler.intakeEnableActions(false);
		}
	}
}
