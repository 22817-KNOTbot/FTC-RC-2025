package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.knotbot.practiceapp.RobotEvent;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BuildConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.HtmlUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.MotorWrapper;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import kotlin.Unit;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "$TeleOp")
public class TeleOp extends LinearOpMode {
	public static boolean DEBUG = true;
	public static int LOGGING_LEVEL = 3; // 0 = no debug, 1 = bulk only, 2 = fast reads, 3 = full reads

	private GamepadManager gamepadManager;
	private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private Automations automationHandler;
	private MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() {
		final int LOGGING_LEVEL = TeleOp.LOGGING_LEVEL;
		boolean manualTurretMode = false;
		boolean manualShooterMode = false;

		gamepadManager = new GamepadManager(gamepad1, gamepad2,
				PanelsGamepad.INSTANCE.getFirstManager()::asCombinedFTCGamepad,
				PanelsGamepad.INSTANCE.getSecondManager()::asCombinedFTCGamepad);
		gamepadManager.updateGamepads();
		gamepad1 = gamepadManager.getGamepad1();
		gamepad2 = gamepadManager.getGamepad2();

		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		// telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
		telemetryManager.setLoggingEnabled(LOGGING_LEVEL > 0);
		telemetryManager.setHtmlMode(true);

		if (gamepad1.right_bumper)
			DEBUG = true;

		Alliance alliance = new RedAlliance();
		if (gamepad1.left_trigger > 0.9) {
			alliance = new BlueAlliance();
		} else if (gamepad1.right_trigger > 0.9) {
			alliance = new RedAlliance();
		} else {
			Object allianceObject = blackboard.getOrDefault("alliance", null);
			if (allianceObject == null) {
				alliance = new RedAlliance();
			} else {
				try {
					alliance = (Alliance) allianceObject;
				} catch (ClassCastException err) {
				}
			}
		}

		final FtcLoggingSession loggingSession = new FtcLoggingSession();
		if (LOGGING_LEVEL > 0) {
			FtcLogTuning.processColorDistanceSensorsInBackground = false;
			FtcLogTuning.pinpointLoggerCallsUpdate = false;
			FtcLogTuning.pedroFollowerPublishesNamedOdometry = true;
			if (LOGGING_LEVEL == 1) {
				FtcLogTuning.bulkOnlyLogging = true;
			}
			if (LOGGING_LEVEL >= 2) {
				FtcLogTuning.bulkOnlyLogging = false;
				FtcLogTuning.logMotorCurrent = true;
				FtcLogTuning.motorCurrentReadPeriodSec = 0.2;
				MotorWrapper.logProfile = MotorWrapper.LOG_PROFILE_FAST;
			}
			if (LOGGING_LEVEL >= 3) {
				MotorWrapper.logProfile = MotorWrapper.LOG_PROFILE_FULL;
			}

			final Alliance currentAlliance = alliance;
			loggingSession.start(this, 5900, "", true, "/sdcard/FIRST/PsiKit/", null, this, () -> {
				String dateString = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date());
				Logger.recordMetadata("Date", dateString);
				Logger.recordMetadata("Alliance", currentAlliance.getColourString());
				Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
				Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
				Logger.recordMetadata("GitDirty", String.valueOf(BuildConstants.DIRTY));
				Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

				return Unit.INSTANCE;
			});
		}

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		// TODO: Change to not reset encoder when running auto
		automationHandler = new Automations(hardwareMap, alliance, true, DEBUG);
		automationHandler.setGamepads(gamepad1, gamepad2);
		mecanumDrive = new MecanumDrive(hardwareMap);

		while (opModeInInit()) {
			Logger.periodicBeforeUser();
			loggingSession.logOncePerLoop(this);

			gamepadManager.updateGamepads();
			gamepad1 = gamepadManager.getGamepad1();
			gamepad2 = gamepadManager.getGamepad2();

			if (gamepad1.left_trigger > 0.9) {
				automationHandler.setAlliance(new BlueAlliance());
			} else if (gamepad1.right_trigger > 0.9) {
				automationHandler.setAlliance(new RedAlliance());
			}
			telemetryManager.addData("Alliance", automationHandler.getAlliance().getColourString());
			if (DEBUG) {
				telemetryManager.addData("Blackboard", blackboard);
			}
			telemetryManager.update();

			Logger.periodicAfterUser(0.0, 0.0);
			idle();
		}

		Object poseObject = blackboard.getOrDefault("pose", null);
		Pose startPose = automationHandler.getAlliance().getResetPose();
		if (poseObject == null) {
			startPose = automationHandler.getAlliance().getResetPose();
		} else {
			try {
				startPose = (Pose) poseObject;
			} catch (ClassCastException err) {
			}
		}
		mecanumDrive.setStartingPose(startPose);
		mecanumDrive.setPose(startPose);

		mecanumDrive.initialize();
		mecanumDrive.setHeadingOffset(automationHandler.getAlliance().getHeadingOffset());
		mecanumDrive.setAutoDriveTarget(automationHandler.getAlliance().getBasePose());
		mecanumDrive.setResetPose(automationHandler.getAlliance().getResetPose());
		automationHandler.start();
		loopTimer.reset();

		RobotEvent.startTeleop();

		while (opModeIsActive()) {
			double beforePsiKitStart = 0;
			double beforeUserEnd = 0;
			if (LOGGING_LEVEL > 0) {
				beforePsiKitStart = Logger.getRealTimestamp();
				Logger.periodicBeforeUser();
				loggingSession.logOncePerLoop(this);
				beforeUserEnd = Logger.getRealTimestamp();
			} else {
				// IMPORTANT: Cache must be cleared every loop to prevent stale data
				// PsiKit already clears bulk cache in FtcLoggingSession.logOncePerLoop()
				for (LynxModule hub : allHubs) {
					hub.clearBulkCache();
				}
			}


			gamepadManager.updateGamepads();
			gamepad1.copy(gamepadManager.getGamepad1());
			gamepad2.copy(gamepadManager.getGamepad2());

			/*
			 * Driver 1
			 */

			if (gamepad1.left_trigger > 0.9) {
				mecanumDrive.move(-gamepad1.left_stick_y / 4, gamepad1.left_stick_x / 4, gamepad1.right_stick_x / 4);
			} else {
				mecanumDrive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			}

			if (getRuntime() >= 100 || DEBUG || gamepad2.right_trigger > 0.9) {
				mecanumDrive.setAutoDrive(gamepad1.left_bumper);
			}
			mecanumDrive.lockingMecanum(gamepad1.right_bumper);
			automationHandler.engageBrakes(gamepad1.right_bumper);

			if (gamepad1.aWasPressed()) {
				automationHandler.intakeToggle();
			}
			if (gamepad1.right_trigger > 0.9) {
				automationHandler.intakeEject();
			} else if (automationHandler.getIntakeEjecting()) {
				automationHandler.intakeEjectStop();
			}

			if (gamepad1.bWasPressed()) {
				Automations.State state = automationHandler.getState();
				if (state != Automations.State.WAITING_TO_SHOOT && state != Automations.State.SHOOTING) {
					automationHandler.startShooting();
				} else {
					automationHandler.setShooting(false);
				}
			}

			Pose currentPose = mecanumDrive.getPose();
			automationHandler.updatePose(currentPose);
			automationHandler.updateVelocity(mecanumDrive.getVelocity());
			automationHandler.automationLoop();

			if (gamepad1.back || gamepad2.back) {
				automationHandler.abort();
			}
			if (gamepad1.startWasPressed() || gamepad2.startWasPressed()) {
				mecanumDrive.resetPose();
			}

			/*
			 * Driver 2
			 */

			if (gamepad2.dpadUpWasPressed()) {
				DEBUG = !DEBUG;
			}
			if (gamepad2.dpadLeftWasPressed()) {
				manualShooterMode = !manualShooterMode;
			}
			if (manualShooterMode) {
				if (gamepad2.xWasPressed()) {
					automationHandler.setShooterVelocity(Shooter.defaultVelocity);
					automationHandler.setShooterEnabled(!automationHandler.getShooterEnabled());
				}
			} else {
				automationHandler.updateShooter();
			}
			if (gamepad2.dpadRightWasPressed()) {
				manualTurretMode = !manualTurretMode;
			}
			if (manualTurretMode) {
				automationHandler.rotateTurret(gamepad2.left_stick_x);
				automationHandler.pitchTurret(gamepad2.right_stick_y);
			} else {
				automationHandler.updateTurret();
			}

			if (gamepad2.left_trigger > 0.9) {
				automationHandler.setIgnoreVelocity(true);
			} else {
				automationHandler.setIgnoreVelocity(false);
			}

			String colour = automationHandler.getAlliance().getColourString();
			telemetryManager.addData("Alliance", HtmlUtil.colourText(colour, colour));
			telemetryManager.addData("Time", getRuntime());
			telemetryManager.addData("State", automationHandler.getState());

			if (manualTurretMode) {
				telemetryManager.addLine(HtmlUtil.colourText("##### MANUAL TURRET MODE #####", "yellow"));
			}
			if (manualShooterMode) {
				telemetryManager.addLine(HtmlUtil.colourText("##### MANUAL SHOOTER MODE #####", "blue"));
			}
			double loopTime = loopTimer.time();
			telemetryManager.addLine(String.format("Loop time: %.2fms - %.0fhz", loopTime, 1000 / loopTime));
			loopTimer.reset();

			if (DEBUG) {
				// Pose holdPose = mecanumDrive.getHoldPose();

				telemetryManager.addData("Pose x", currentPose.getX());
				telemetryManager.addData("Pose y", currentPose.getY());
				telemetryManager.addData("Pose heading", Math.toDegrees(currentPose.getHeading()));

				// telemetryManager.addData("Hold Position X", holdPose.getX());
				// telemetryManager.addData("Hold Position Y", holdPose.getY());

				// telemetryManager.addData("Auto driving", mecanumDrive.isAutoDrive());
				// telemetryManager.addData("Auto drive target", mecanumDrive.getAutoDriveTarget());
				telemetryManager.addData("Shooter Velocity", automationHandler.getShooterVelocity());
				telemetryManager.addData("Shooter Desired Velocity", automationHandler.getShooterDesiredVelocity());

				automationHandler.showTelemetry(telemetryManager);

				Drawing.drawRobot(currentPose, telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();
			}
			telemetryManager.update();

			if (LOGGING_LEVEL > 0) {
				Logger.recordOutput("LoopTime", loopTime);
				Pose ftcPose = currentPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
				Pose2d wpiPose = new Pose2d(DistanceUnit.INCH.toMeters(ftcPose.getX()),
						DistanceUnit.INCH.toMeters(ftcPose.getY()), Rotation2d.fromRadians(ftcPose.getHeading()));
				Logger.recordOutput("Pose", wpiPose);
			}

			if (LOGGING_LEVEL > 0) {
				Logger.periodicAfterUser(
						Logger.getRealTimestamp() - beforeUserEnd,
						beforeUserEnd - beforePsiKitStart);
			}
		}

		automationHandler.end();
		RobotEvent.runEnd();
		loggingSession.end();
	}
}
