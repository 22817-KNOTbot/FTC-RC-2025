package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

import java.util.List;

@Configurable
public class TeleOp extends LinearOpMode {
	public static boolean DEBUG = false;

	private GamepadManager gamepadManager;
	private ElapsedTime runtime = new ElapsedTime();
	private Automations automationHandler;
	private MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() {
		gamepadManager = new GamepadManager(gamepad1, gamepad2,
				PanelsGamepad.INSTANCE.getFirstManager()::asCombinedFTCGamepad,
				PanelsGamepad.INSTANCE.getSecondManager()::asCombinedFTCGamepad);
		gamepadManager.updateGamepads();
		gamepad1.copy(gamepadManager.getGamepad1());
		gamepad2.copy(gamepadManager.getGamepad2());

		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
		if (gamepad1.right_bumper)
			DEBUG = true;

		Alliance alliance;
		if (gamepad1.right_trigger > 0.9) {
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
			if (gamepad1.right_trigger > 0.9) {
				automationHandler.setAlliance(new BlueAlliance());
			} else if (gamepad1.right_trigger > 0.9) {
				automationHandler.setAlliance(new RedAlliance());
			}
			telemetry.addData("Alliance", automationHandler.getAlliance().getColourString());
		}

		mecanumDrive.initialize();
		mecanumDrive.setHeadingOffset(automationHandler.getAlliance().getHeadingOffset());
		runtime.reset();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			gamepadManager.updateGamepads();
			gamepad1.copy(gamepadManager.getGamepad1());
			gamepad2.copy(gamepadManager.getGamepad2());

			mecanumDrive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			mecanumDrive.lockingMecanum(gamepad1.left_bumper);

			if (gamepad1.aWasPressed()) {
				automationHandler.intakeToggle();
			}
			if (gamepad1.yWasPressed()) {
				automationHandler.shootArtifact(Artifact.Colour.PURPLE);
			} else if (gamepad1.bWasPressed()) {
				automationHandler.shootArtifact(Artifact.Colour.GREEN);
			}

			automationHandler.updatePose(mecanumDrive.getPose());
			automationHandler.automationLoop();

			if (gamepad1.back || gamepad2.back) {
				automationHandler.abort();
			} else if (gamepad1.start || gamepad2.start) {
				mecanumDrive.resetPose();
			}

			telemetry.addData("Alliance", automationHandler.getAlliance().getColourString());
			telemetry.addData("Time", runtime.time());
			telemetry.addData("Storage State", automationHandler.getStorageState());
			if (!automationHandler.colourSensorResponding()) {
				telemetry.addLine("********************");
				telemetry.addLine("WARNING: COLOUR SENSOR");
				telemetry.addLine("IS NOT RESPONDING");
				telemetry.addLine("********************");
			}
			if (DEBUG) {
				Pose pose = mecanumDrive.getPose();
				Pose holdPose = mecanumDrive.getHoldPose();

				telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));

				telemetry.addData("Position X", pose.getX());
				telemetry.addData("Position Y", pose.getY());

				telemetry.addData("Hold Position X", holdPose.getX());
				telemetry.addData("Hold Position Y", holdPose.getY());
				automationHandler.showTelemetry(telemetry);
			}
			telemetry.update();
		}
	}
}
