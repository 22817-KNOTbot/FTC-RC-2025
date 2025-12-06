package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.knotbot.practiceapp.RobotEvent;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
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

@Configurable
@Config
@Autonomous(name = "Lower Blue 9 Artifacts", group = "Autonomous")
public class BlueLow9Artifacts extends LinearOpMode {
	public static boolean doMovement = true;
	public static boolean doActions = true;
	public static boolean DEBUG = true;
	public static double intakeMaxPower = 0.25;
	public static double shooterVelocityTimeout = 5;

	private boolean intaking = false;
	private boolean shooting = false;
	private int pathState = 0;
	private int shots = 0;
	private boolean manuallyMovedTurret = false;

	private Alliance alliance = new BlueAlliance();
	private Automations automationHandler;
	private Follower follower;
	private Pose startPose = new Pose(96.000, 9.000, Math.toRadians(0)).mirror();
	private Artifact.Colour[] patternColours = null;
	private ElapsedTime shootingTimer = new ElapsedTime();
	private ElapsedTime stateTimer = new ElapsedTime();
	private boolean shootingTimerReset = false;

	private boolean actionInit = false;
	private boolean actionDone = false;

	private PathChain firstApproach, firstIntake, firstLaunch,
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
		automationHandler.setArtifactsStored(new Colour[] {Colour.GREEN, Colour.PURPLE, Colour.PURPLE});
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
			telemetryManager.addData("Motif", automationHandler.getArtifactPattern());
			telemetryManager.update();
		}

		automationHandler.start();
		stateTimer.reset();

		RobotEvent.startAuto();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			if (doActions) {
				automationHandler.updatePose(follower.getPose());
				automationHandler.updateVelocity(follower.getVelocity());
				// automationHandler.updateTurret();
				automationHandler.updateShooter();

				if (shooting && shootingTimerReset && shootingTimer.time() >= shooterVelocityTimeout) {
					automationHandler.setIgnoreVelocity(true);
				}
				automationHandler.automationLoop();
				automationHandler.setIgnoreVelocity(false);
				if (intaking) {
					if (automationHandler.getStorageState() == Automations.StorageState.WAITING) {
						intaking = false;
						actionDone = true;
					}
				} else if (shooting) {
					if (automationHandler.getStorageState() == Automations.StorageState.WAITING) {
						shooting = false;
						shootingTimerReset = false;
						actionDone = true;
						manuallyMovedTurret = false;
						shots++;
					} else if (automationHandler.getTransferState() == Storage.TransferState.RAMP_OUT) {
						if (!manuallyMovedTurret) {
							switch (shots) {
								case 0:
									automationHandler.setTurretRotationDegrees(Math.toDegrees(Math.atan2(135, 48)));
									break;
								case 1:
								case 2:
									automationHandler.setTurretRotationDegrees(-Math.toDegrees(Math.atan2(48, 133)) - 5);
									// break;
									// automationHandler.setTurretRotationDegrees(Math.toDegrees(Math.atan2(48, 133)));
									break;
								default:
									break;

							}
							manuallyMovedTurret = true;
						}
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

			telemetryManager.addData("Path state", pathState);
			telemetryManager.addData("Pose", follower.getPose());
			telemetryManager.addData("Follower busy", follower.isBusy());
			telemetryManager.addData("Action done", actionDone);
			telemetryManager.addData("Shooting", shooting);
			telemetryManager.addData("Storage", automationHandler.getArtifactsStored());
			telemetryManager.addData("Storage State", automationHandler.getStorageState());
			telemetryManager.addData("Storage Intake State", automationHandler.getIntakeState());
			telemetryManager.addData("Storage Transfer State", automationHandler.getTransferState());
			telemetryManager.addData("Shooter Velocity", automationHandler.getShooterVelocity());
			telemetryManager.addData("Shooter Desired Velocity", automationHandler.getShooterDesiredVelocity());

			if (DEBUG) {
				Drawing.drawRobot(follower.getPose(), telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();

				telemetryManager.addData("T value", follower.getCurrentTValue());
				telemetryManager.addData("Path completion", follower.getPathCompletion());
				if (doActions) {
					automationHandler.showTelemetry(telemetryManager);
				}

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
		firstApproach = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(96.000, 9.000).mirror(), new Pose(96.000, 36.000).mirror()))
				.setConstantHeadingInterpolation(Math.toRadians(180 - 0))
				.setTValueConstraint(0.73)
				.setBrakingStrength(0.4)
				// .addParametricCallback(0, this::readyToShoot)
				// .addParametricCallback(0, this::startShooting)
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(96.000, 36.000).mirror(), new Pose(140.500, 36.000).mirror()))
				.setConstantHeadingInterpolation(Math.toRadians(180 - 0))
				// .addParametricCallback(0, this::intakeToggle)
				// .addParametricCallback(1, this::intakeToggle)
				.addParametricCallback(0, this::intakeEnable)
				.addParametricCallback(0.05, this::startIntakeSpeed)
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(140.500, 36.000).mirror(),
								new Pose(96.000, 36.000).mirror(),
								new Pose(96.000, 11.000).mirror()))
				.setTangentHeadingInterpolation()
				.setReversed()
				// .addParametricCallback(1, this::startShooting)
				.addParametricCallback(0.9, this::intakeDisable)
				.build();

		secondApproach = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(96.000, 11.000).mirror(), new Pose(96.000, 58.300).mirror()))
				.setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(180 - 0))
				.setTValueConstraint(0.8)
				.setBrakingStrength(0.4)
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(96.000, 58.300).mirror(),
								new Pose(120.000, 56.900).mirror(),
								new Pose(140.500, 56.300).mirror()))
				.setConstantHeadingInterpolation(Math.toRadians(180 - 0))
				.addParametricCallback(0, this::intakeEnable)
				.addParametricCallback(0.05, this::startIntakeSpeed)
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(140.500, 56.300).mirror(),
								new Pose(96.000, 56.900).mirror(),
								new Pose(96.000, 11.000).mirror()))
				.setTangentHeadingInterpolation()
				.setReversed()
				// .addParametricCallback(1, this::startShooting)
				.addParametricCallback(0.9, this::intakeDisable)
				.build();

		exitShootingZone = follower.pathBuilder()
				.addPath(
						new BezierLine(new Pose(96.000, 11.000).mirror(), new Pose(96.000, 36.000).mirror()))
				.setTangentHeadingInterpolation()
				.build();
	}

	public void setPathState(int state) {
		stateTimer.reset();
		actionDone = !doActions;
		actionInit = !doActions;
		pathState = state;
	}

	public void pathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy()) {
					if (!actionDone) {
						if (!actionInit) {
							readyToShoot();
							startShooting();
							actionInit = true;
						}
					} else {
						follower.followPath(firstApproach, true);
						setPathState(1);
					}
				}
				break;
			case 1:
				if (follower.atParametricEnd()) {
					follower.followPath(firstIntake, true);
					setPathState(2);
				}
				break;
			case 2:
				if (actionDone && (!follower.isBusy() || (stateTimer.time() > 1.5 && follower.getVelocity().getMagnitude() < 0.2))) {
					follower.setMaxPower(1);
					follower.followPath(firstLaunch, true);
					setPathState(3);
				}
				break;
			case 3:
				if (!follower.isBusy()) {
					if (!actionDone) {
						if (!actionInit) {
							startShooting();
							actionInit = true;
						}
					} else {
						follower.followPath(secondApproach, true);
						setPathState(4);
					}
				}
				break;
			case 4:
				if (follower.atParametricEnd()) {
					follower.followPath(secondIntake, true);
					setPathState(5);
				}
				break;
			case 5:
				if (!follower.isBusy() || (stateTimer.time() > 1.5 && follower.getVelocity().getMagnitude() < 0.2)) {
					follower.setMaxPower(1);
					follower.followPath(secondLaunch, true);
					setPathState(6);
				}
				break;
			case 6:
				if (!follower.isBusy()) {
					if (!actionDone) {
						if (!actionInit) {
							startShooting();
							actionInit = true;
						}
					} else {
						follower.followPath(exitShootingZone, true);
						setPathState(-1);
					}
				}
				break;
		}
	}

	public void readyToShoot() {
		if (doActions) {
			// automationHandler.setIgnoreVelocity(true);
			// automationHandler.setTransferMode(Storage.TransferMode.FULL_SPIN);
			Pattern pattern = automationHandler.getArtifactPattern();
			if (pattern != null) {
				patternColours = pattern.getPattern();
			}
			automationHandler.setRapidFire(true);
			automationHandler.setUseVision(false);
		}
	}

	public void startShooting() {
		if (doActions) {
			// follower.pausePathFollowing();
			// if (firstShot) {
			// 	automationHandler.setUseVision(false);
			// } else {
			// 	automationHandler.setUseVision(true);
			// }
			if (patternColours == null || automationHandler.prepareOrShootArtifactSequence(patternColours) == Storage.TurnDirection.NONE) {
				automationHandler.shootActiveArtifact(true);
				// automationHandler.prepareOrShootAnyArtifact();
			};
			shooting = true;
		}
	}

	public void intakeEnable() {
		if (doActions) {
			intaking = true;
			automationHandler.intakeEnableActions(true);
		}
	}

	public void intakeDisable() {
		if (doActions) {
			intaking = false;
			// automationHandler.intakeEnableActions(false);
			automationHandler.intakeEnable(false);
		}
	}

	public void startIntakeSpeed() {
		follower.setMaxPower(intakeMaxPower);
	}
}