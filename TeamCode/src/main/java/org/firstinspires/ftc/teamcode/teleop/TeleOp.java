package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.List;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {
	public static boolean DEBUG = true;
	public static boolean pedroLocalizer = true; // Roadrunner if false

	private GamepadManager gamepadManager;
	private ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private Automations automationHandler;
	private MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() {
		boolean manualTurretMode = false;
		boolean manualShooterMode = false;

		gamepadManager = new GamepadManager(gamepad1, gamepad2,
				PanelsGamepad.INSTANCE.getFirstManager()::asCombinedFTCGamepad,
				PanelsGamepad.INSTANCE.getSecondManager()::asCombinedFTCGamepad);
		gamepadManager.updateGamepads();
		gamepad1.copy(gamepadManager.getGamepad1());
		gamepad2.copy(gamepadManager.getGamepad2());

		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		if (gamepad1.right_bumper)
			DEBUG = true;

		Alliance alliance;
		if (gamepad1.left_trigger > 0.9) {
			alliance = new BlueAlliance();
		} else if (gamepad1.right_trigger > 0.9) {
			alliance = new RedAlliance();
		} else {
			alliance = (Alliance) blackboard.getOrDefault("alliance", new RedAlliance());
		}

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		automationHandler = new Automations(hardwareMap, alliance, DEBUG);
		automationHandler.setGamepads(gamepad1, gamepad2);
		mecanumDrive = new MecanumDrive(hardwareMap);

		while (opModeInInit()) {
			gamepadManager.updateGamepads();
			gamepad1.copy(gamepadManager.getGamepad1());
			gamepad2.copy(gamepadManager.getGamepad2());

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
		}
		
		Pose pose = (Pose) blackboard.getOrDefault("pose", automationHandler.getAlliance().getResetPose());
		mecanumDrive.setPose(pose);

		mecanumDrive.initialize();
		mecanumDrive.setHeadingOffset(automationHandler.getAlliance().getHeadingOffset());
		mecanumDrive.setAutoDriveTarget(automationHandler.getAlliance().getBasePose());
		mecanumDrive.setResetPose(automationHandler.getAlliance().getResetPose());
		automationHandler.start();
		loopTime.reset();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			gamepadManager.updateGamepads();
			gamepad1.copy(gamepadManager.getGamepad1());
			gamepad2.copy(gamepadManager.getGamepad2());

			if (gamepad1.left_trigger > 0.9) {
				mecanumDrive.move(-gamepad1.left_stick_y / 4, gamepad1.left_stick_x / 4, gamepad1.right_stick_x / 4);
			} else {
				mecanumDrive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			}

			if (getRuntime() >= 100 || DEBUG || gamepad2.right_trigger > 0.9) {
				mecanumDrive.setAutoDrive(gamepad1.right_bumper);
			}
			mecanumDrive.lockingMecanum(gamepad1.left_bumper);

			if (gamepad1.aWasPressed()) {
				automationHandler.intakeToggle();
			}
			if (gamepad1.right_trigger > 0.9) {
				automationHandler.intakeEject();
			} else if (automationHandler.getIntakeEjecting()) {
				automationHandler.intakeEjectStop();
			}

			if (gamepad1.xWasPressed()) {
				boolean newRapidFireState = !automationHandler.getRapidFire();
				automationHandler.setRapidFire(newRapidFireState);
				automationHandler.vibrateControllersBlips(newRapidFireState ? 2 : 1);
			}
			if (!automationHandler.getRapidFire()) {
				if (gamepad1.bWasPressed()) {
					automationHandler.prepareOrShootArtifact(Artifact.Colour.PURPLE);
				} else if (gamepad1.yWasPressed()) {
					automationHandler.prepareOrShootArtifact(Artifact.Colour.GREEN);
				}
			} else {
				if (gamepad1.yWasPressed() || gamepad1.bWasPressed()) {
					automationHandler.prepareOrShootAnyArtifact();
				}
			}

			automationHandler.updatePose(mecanumDrive.getPose());
			automationHandler.updateVelocity(mecanumDrive.getVelocity());
			automationHandler.automationLoop();

			if (gamepad1.back || gamepad2.back) {
				automationHandler.abort();
			} else if (gamepad1.start || gamepad2.start) {
				mecanumDrive.resetPose();
			}

			/*
			 * Driver 2 overrides
			 */

			if (gamepad2.aWasPressed()) {
				automationHandler.intakeToggle();
			}
			if (gamepad2.xWasPressed()) {
				automationHandler.shootActiveArtifact(true);
			}
			if (gamepad2.rightBumperWasPressed()) {
				automationHandler.storageTurnCW();
			} else if (gamepad2.leftBumperWasPressed()) {
				automationHandler.storageTurnCCW();
			}
			if (gamepad2.dpadUpWasPressed()) {
				DEBUG = !DEBUG;
			}
			if (gamepad2.leftStickButtonWasPressed()) {
				automationHandler.clearStorageMemory();
			}

			if (gamepad2.dpadDownWasPressed()) {
				manualTurretMode = !manualTurretMode;
			}
			if (manualTurretMode) {
				automationHandler.rotateTurret(gamepad2.left_stick_x);
				automationHandler.pitchTurret(gamepad2.right_stick_y);
			} else {
				automationHandler.updateTurret();
			}
			if (gamepad2.yWasPressed()) {
				manualShooterMode = !manualShooterMode;
			}
			if (manualShooterMode) {
				if (gamepad2.bWasPressed()) {
					automationHandler.setShooterVelocity(Shooter.defaultVelocity);
					automationHandler.setShooterEnabled(!automationHandler.getShooterEnabled());
				}
			} else {
				automationHandler.updateShooter();
			}

			telemetryManager.addData("Alliance", automationHandler.getAlliance().getColourString());
			telemetryManager.addData("Pattern", automationHandler.getPattern());
			telemetryManager.addData("Time", getRuntime());
			telemetryManager.addData("Rapid Fire", automationHandler.getRapidFire());
			telemetryManager.addData("Storage State", automationHandler.getStorageState());
			if (!automationHandler.colourSensorResponding()) {
				telemetryManager.addLine("********************");
				telemetryManager.addLine("WARNING: COLOUR SENSOR");
				telemetryManager.addLine("IS NOT RESPONDING");
				telemetryManager.addLine("********************");
			}
			if (manualTurretMode) {
				telemetryManager.addLine("##### MANUAL TURRET MODE #####");
			}
			if (manualShooterMode) {
				telemetryManager.addLine("##### MANUAL SHOOTER MODE #####");
			}
			telemetryManager.addLine(String.format("Loop time: %.2fms - %.0fhz", loopTime.time(), 1000 / loopTime.time()));
			loopTime.reset();

			if (DEBUG) {
				Pose currentPose = mecanumDrive.getPose();
				// Pose holdPose = mecanumDrive.getHoldPose();

				telemetryManager.addData("Heading", Math.toDegrees(currentPose.getHeading()));

				telemetryManager.addData("Position X", currentPose.getX());
				telemetryManager.addData("Position Y", currentPose.getY());

				// telemetryManager.addData("Hold Position X", holdPose.getX());
				// telemetryManager.addData("Hold Position Y", holdPose.getY());

				telemetryManager.addData("Auto driving", mecanumDrive.isAutoDrive());
				// telemetryManager.addData("Auto drive target", mecanumDrive.getAutoDriveTarget());
				automationHandler.showTelemetry(telemetryManager);

				Drawing.drawRobot(currentPose, telemetryManager.getDashboardCanvas());
				Drawing.drawRobot(mecanumDrive.getRrPose(), new Style("", "#b33232", 0.75), telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();

			}
			telemetryManager.update();
		}

		automationHandler.end();
	}
}
