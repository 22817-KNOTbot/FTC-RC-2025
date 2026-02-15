package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Log;
import gay.zharel.fateweaver.flight.FlightLogChannel;
import gay.zharel.fateweaver.flight.FlightRecorder;
import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;
import gay.zharel.fateweaver.schemas.FateSchema;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.knotbot.practiceapp.RobotEvent;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.BuildConstants;
import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.HtmlUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.FateWeaver.CustomSchemas;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.function.Function;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "$TeleOp")
public class TeleOp extends LinearOpMode {
	public static boolean DEBUG = true;
	public static boolean LOGGING_ENABLED = true;

	private GamepadManager gamepadManager;
	private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
		gamepad1 = gamepadManager.getGamepad1();
		gamepad2 = gamepadManager.getGamepad2();

		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		// telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
		telemetryManager.setLoggingEnabled(false);
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
				} catch (ClassCastException err) {}
			}
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

		FlightLogChannel<Double> runTimeChannel = FlightRecorder.createChannel("RunTime", Double.class);
		FlightLogChannel<Double> loopTimesChannel = FlightRecorder.createChannel("LoopTime", Double.class);
		FlightLogChannel<String> gamepad1Channel = FlightRecorder.createChannel("Gamepad/1", String.class);
		FlightLogChannel<String> gamepad2Channel = FlightRecorder.createChannel("Gamepad/2", String.class);
		FlightLogChannel<Double> poseXChannel = FlightRecorder.createChannel("Pose/x", Double.class);
		FlightLogChannel<Double> poseYChannel = FlightRecorder.createChannel("Pose/y", Double.class);
		FlightLogChannel<Double> poseHeadingChannel = FlightRecorder.createChannel("Pose/heading", Double.class);
		FlightLogChannel<Double> velocityXChannel = FlightRecorder.createChannel("Velocity/x", Double.class);
		FlightLogChannel<Double> velocityYChannel = FlightRecorder.createChannel("Velocity/y", Double.class);
		FlightLogChannel<Double> velocityMagnitudeChannel = FlightRecorder.createChannel("Velocity/magnitude", Double.class);
		FlightLogChannel<Double> velocityThetaChannel = FlightRecorder.createChannel("Velocity/theta", Double.class);
		
		if (LOGGING_ENABLED) {
			String dateString = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date());
			FlightRecorder.write("/Metadata/Date", dateString);
			FlightRecorder.write("/Metadata/OpMode", this.getClass().getSimpleName());
			FlightRecorder.write("/Metadata/GitSHA", BuildConstants.GIT_SHA);
			FlightRecorder.write("/Metadata/GitBranch", BuildConstants.GIT_BRANCH);
			FlightRecorder.write("/Metadata/GitDirty", String.valueOf(BuildConstants.DIRTY));
			FlightRecorder.write("/Metadata/BuildDate", BuildConstants.BUILD_DATE);
			FlightRecorder.write("/Info/Alliance", alliance.getColourString());
		}

		while (opModeInInit()) {
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
		}
		
		Object poseObject = blackboard.getOrDefault("pose", null);
		Pose startPose = automationHandler.getAlliance().getResetPose();
		if (poseObject == null) {
			startPose = automationHandler.getAlliance().getResetPose();
		} else {
			try {
				startPose = (Pose) poseObject;
			} catch (ClassCastException err) {}
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
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
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
				automationHandler.shootActiveArtifact(true);
			}

			Pose pose = mecanumDrive.getPose();
			Vector velocity = mecanumDrive.getVelocity();
			automationHandler.updatePose(pose);
			automationHandler.updateVelocity(velocity);
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

			if (gamepad2.yWasPressed()) {
				automationHandler.prepareArtifactSequence(Artifact.Pattern.GPP.getPattern(), false);
			}
			if (gamepad2.bWasPressed()) {
				automationHandler.prepareArtifactSequence(Artifact.Pattern.PGP.getPattern(), false);
			}
			if (gamepad2.aWasPressed()) {
				automationHandler.prepareArtifactSequence(Artifact.Pattern.PPG.getPattern(), false);
			}

			if (gamepad2.rightBumperWasPressed()) {
				automationHandler.storageTurnCW();
			} else if (gamepad2.leftBumperWasPressed()) {
				automationHandler.storageTurnCCW();
			}
			if (gamepad2.leftStickButtonWasPressed()) {
				automationHandler.clearStorageMemory();
			}

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
			for (String line : buildStorageTelemetryDisplay(automationHandler.getArtifactsStored())) {
				telemetryManager.addData("", line);
			}
			telemetryManager.addData("Storage State", automationHandler.getStorageState());
			telemetryManager.addData("Storage Intake State", automationHandler.getIntakeState());
			telemetryManager.addData("Storage Transfer State", automationHandler.getTransferState());

			if (!automationHandler.colourSensorsRespondingLazy()) {
				telemetryManager.addLine(HtmlUtil.colourText("********************", "red"));
				telemetryManager.addLine(HtmlUtil.colourText("WARNING: COLOUR SENSOR(S)", "red"));
				telemetryManager.addLine(HtmlUtil.colourText("NOT RESPONDING", "red"));
				telemetryManager.addLine(HtmlUtil.colourText("********************", "red"));
			}

			if (manualTurretMode) {
				telemetryManager.addLine(HtmlUtil.colourText("##### MANUAL TURRET MODE #####", "yellow"));
			}
			if (manualShooterMode) {
				telemetryManager.addLine(HtmlUtil.colourText("##### MANUAL SHOOTER MODE #####", "orange"));
			}
			double loopTime = loopTimer.time();
			telemetryManager.addLine(String.format("Loop time: %.2fms - %.0fhz", loopTime, 1000 / loopTime));
			loopTimer.reset();

			if (DEBUG) {
				// Pose holdPose = mecanumDrive.getHoldPose();

				telemetryManager.addData("Pose x", pose.getX());
				telemetryManager.addData("Pose y", pose.getY());
				telemetryManager.addData("Pose heading", Math.toDegrees(pose.getHeading()));

				Pose ftcPose = pose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
				telemetryManager.addData("FTC Pose x", ftcPose.getX());
				telemetryManager.addData("FTC Pose y", ftcPose.getY());
				telemetryManager.addData("FTC Pose heading (deg)", Math.toDegrees(ftcPose.getHeading()));

				// telemetryManager.addData("Hold Position X", holdPose.getX());
				// telemetryManager.addData("Hold Position Y", holdPose.getY());

				// telemetryManager.addData("Auto driving", mecanumDrive.isAutoDrive());
				// telemetryManager.addData("Auto drive target", mecanumDrive.getAutoDriveTarget());
				telemetryManager.addData("Shooter Velocity", automationHandler.getShooterVelocity());
				telemetryManager.addData("Shooter Desired Velocity", automationHandler.getShooterDesiredVelocity());
	
				automationHandler.showTelemetry(telemetryManager);

				Drawing.drawRobot(pose, telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();
			}
			telemetryManager.update();

			if (LOGGING_ENABLED) {
				runTimeChannel.put(getRuntime());
				loopTimesChannel.put(loopTime);
				gamepad1Channel.put(gamepad1.toString());
				gamepad2Channel.put(gamepad2.toString());
				poseXChannel.put(pose.getX());
				poseYChannel.put(pose.getY());
				poseHeadingChannel.put(pose.getHeading());
				velocityXChannel.put(velocity.getXComponent());
				velocityYChannel.put(velocity.getYComponent());
				velocityMagnitudeChannel.put(velocity.getMagnitude());
				velocityThetaChannel.put(velocity.getTheta());
			}
		}

		automationHandler.end();

		RobotEvent.runEnd();
	}

	private String[] buildStorageTelemetryDisplay(List<Artifact.Colour> colours) {
		String[] lines = new String[4];

		String activeString = "";
		if (colours.get(0) == null) {
			activeString = "  ";
		} else {
			switch (colours.get(0)) {
				case PURPLE:
					activeString = HtmlUtil.colourText("PP", "#cc00ff");
					break;
				case GREEN:
					activeString = HtmlUtil.colourText("GG", "green");
					break;
				default:
					activeString = "  ";
					break;
			}
		}

		String backLeftString = "";
		if (colours.get(1) == null) {
			backLeftString = "  ";
		} else {
			switch (colours.get(1)) {
				case PURPLE:
					backLeftString = HtmlUtil.colourText("PP", "#cc00ff");
					break;
				case GREEN:
					backLeftString = HtmlUtil.colourText("GG", "green");
					break;
				default:
					backLeftString = "  ";
					break;
			}
		}

		String backRightString = "";
		if (colours.get(2) == null) {
			backRightString = "  ";
		} else {
			switch (colours.get(2)) {
				case PURPLE:
					backRightString = HtmlUtil.colourText("PP", "#cc00ff");
					break;
				case GREEN:
					backRightString = HtmlUtil.colourText("GG", "green");
					break;
				default:
					backRightString = "  ";
					break;
			}
		}

		lines[0] = lines[1] = HtmlUtil.monospaceText("&nbsp&nbsp" + activeString + "&nbsp&nbsp");
		lines[2] = lines[3] = HtmlUtil.monospaceText(backLeftString + "&nbsp&nbsp" + backRightString);
		return lines;
	}
}
