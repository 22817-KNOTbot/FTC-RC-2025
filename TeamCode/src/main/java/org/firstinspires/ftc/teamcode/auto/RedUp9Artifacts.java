package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;
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
	public static boolean doActions = true;
	public static boolean DEBUG = true;
	public static double intakeMaxPower = 0.25;
	public static double shooterVelocityTimeout = 5;

	private boolean intaking = false;
	private boolean shooting = false;
	private int pathState = 0;

	private Alliance alliance = new RedAlliance();
	private Automations automationHandler;
	private Follower follower;
	private Pose startPose = new Pose(123.000, 124.000, Math.toRadians(125));
	private Artifact.Colour[] patternColours = null;
	private ElapsedTime shootingTimer = new ElapsedTime();
	private ElapsedTime stateTimer = new ElapsedTime();
	private boolean shootingTimerReset = false;

	private boolean actionInit = false;
	private boolean actionDone = false;

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
				automationHandler.showTelemetry(telemetryManager);

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
				// .addParametricCallback(0.9, this::readyToShoot)
				// .addParametricCallback(0.9, this::startShooting)
				.setBrakingStrength(0.2)
				.setVelocityConstraint(5)
				.setHeadingConstraint(0.2)
				.setTranslationalConstraint(1)
				.setTimeoutConstraint(1000)
				.build();

		firstIntake = follower.pathBuilder()
				.addPath(new BezierLine(new Pose(84.000, 84.000), new Pose(124.000, 83.000)))
				.setTangentHeadingInterpolation()
				.addParametricCallback(0, this::intakeEnable)
				// .addParametricCallback(1, this::intakeDisable)
				// .addPoseCallback(new Pose(100, 80), this::startIntakeSpeed, 0.4)
				// .setBrakingStrength(1)
				.addParametricCallback(0.03, this::startIntakeSpeed)
				.build();

		firstLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(124.000, 83.000),
								new Pose(100.000, 84.000),
								new Pose(84.000, 84.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(0.9, this::intakeDisable)
				// .addParametricCallback(1, this::startShooting)
				.setBrakingStrength(0.2)
				.setVelocityConstraint(5)
				.setHeadingConstraint(0.2)
				.setTranslationalConstraint(1)
				.setTimeoutConstraint(1000)
				.build();

		secondApproach = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 84.000),
								new Pose(85.000, 60.000),
								// new Pose(90.000, 60.000),
								new Pose(96.000, 56.300)))
				// .setTangentHeadingInterpolation()
				.setConstantHeadingInterpolation(0)
				// .setBrakingStrength(1)
				.build();

		secondIntake = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(96.000, 56.300),
								new Pose(120.000, 55.900),
								new Pose(128.500, 55.300)))
				.setConstantHeadingInterpolation(Math.toRadians(0))
				.addParametricCallback(0, this::intakeEnable)
				.addParametricCallback(0, this::startIntakeSpeed)
				// .addParametricCallback(0.9, this::intakeDisable)
				// .setBrakingStrength(1)
				.build();

		secondLaunch = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(128.500, 55.300),
								new Pose(120.000, 55.000),
								new Pose(84.000, 84.000)))
				.setTangentHeadingInterpolation()
				.setReversed()
				.addParametricCallback(0.9, this::intakeDisable)
				// .addParametricCallback(0.9, this::startShooting)
				.setBrakingStrength(0.2)
				.setVelocityConstraint(5)
				.setHeadingConstraint(0.2)
				.setTranslationalConstraint(1)
				.setTimeoutConstraint(1000)
				.build();

		exitShootingZone = follower
				.pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(84.000, 84.000),
								new Pose(85.000, 76.000),
								new Pose(85.000, 34.000),
								new Pose(95.000, 34.000)))
				.setConstantHeadingInterpolation(0)
				// .setBrakingStrength(1)
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
					follower.followPath(preloadLaunch, true);
					setPathState(1);
				}
				break;
			case 1:
				if (!follower.isBusy()) {
					if (!actionDone) {
						if (!actionInit) {
							readyToShoot();
							startShooting();
							actionInit = true;
						}
					} else {
						// follower.setMaxPower(intakeMaxPower);
						follower.followPath(firstIntake, true);
						setPathState(2);
					}
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
						follower.followPath(secondApproach, false);
						setPathState(4);
					}
				}
				break;
			case 4:
				if (follower.atParametricEnd()) {
					// follower.setMaxPower(intakeMaxPower);
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
			// automationHandler.setIgnoreVelocity(true);
			automationHandler.setTransferMode(Storage.TransferMode.FULL_SPIN);
			Pattern pattern = automationHandler.getArtifactPattern();
			if (pattern != null) {
				patternColours = pattern.getPattern();
			}
			automationHandler.setRapidFire(true);
		}
	}

	public void startShooting() {
		if (doActions) {
			// follower.pausePathFollowing();
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
