package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoAlign.AlignmentDirection;

public class Automations {
	private HardwareMap hardwareMap;
	private boolean DEBUG;
	private StorageState storageState;
	private ElapsedTime timer;

	private Intake intake;
	private Storage storage;
	private Turret turret;
	private Shooter shooter;
	private Vision vision;

	private Gamepad gamepad1;
	private Gamepad gamepad2;

	private Artifact.Pattern pattern;
	private boolean intakeEnabled;

	public enum StorageState {
		WAITING,
		STORING,
		RELEASING
	}

	public Automations(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public Automations(HardwareMap hardwareMap, boolean DEBUG) {
		this.hardwareMap = hardwareMap;
		this.DEBUG = DEBUG;
		this.storageState = StorageState.WAITING;
		this.timer = new ElapsedTime();
		intake = new Intake(hardwareMap);
		storage = new Storage(hardwareMap);
		turret = new Turret(hardwareMap);
		shooter = new Shooter(hardwareMap);

		Vision.DEBUG = DEBUG;
		vision = new Vision(hardwareMap, null);
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	public void showTelemetry(Telemetry telemetry) {
		telemetry.addData("Intake enabled", intakeEnabled);
		vision.showTelemetry(telemetry);
	}

	public void abort() {
		// Exists for future use
	}

	// Should be called every loop. Handles various things
	// that need to be called repeatedly
	public void automationLoop() {
		vision.updateMotifPattern();
		pattern = vision.getLastMotifPattern();
		if (storageState == StorageState.WAITING && storage.intake()) {
			storage.storeArtifact();
			timer.reset();
			storageState = StorageState.STORING;
		} else if (storageState == StorageState.STORING && timer.time() > 0.5) {
			storageState = StorageState.WAITING;
		} else if (storageState == StorageState.RELEASING && timer.time() > 0.5) {
			shooter.enable(false);
			storageState = StorageState.WAITING;
		}

		AlignmentDirection direction = vision.getAlignmentDirection();
		if (direction.directionKnown) {
			turret.rotateTurret(direction.x);
			turret.setPitch(Range.scale(direction.y, -1, 1, Turret.min_pitch, Turret.max_pitch));
		}
	}

	public void intakeToggle() {
		intake.enable(!intakeEnabled);
		intakeEnabled = !intakeEnabled;
	}

	public void shootArtifact(Artifact.Colour colour) {
		if (storage.getFrontLeftArtifact() == colour) {
			storage.releaseLeft();
		} else if (storage.getFrontRightArtifact() == colour) {
			storage.releaseRight();
		} else {
			vibrateControllers();
			return;
		}
		shooter.enable(true);
	}

	public boolean colourSensorResponding() {
		return storage.colourSensorResponding();
	}

	/*
	 * Getter methods
	 */

	public StorageState getStorageState() {
		return storageState;
	}

	/*
	 * Misc. util methods
	 */

	public void vibrateControllers() {
		vibrateControllers(100);
	}

	public void vibrateControllers(int durationMs) {
		if (gamepad1 != null)
			gamepad1.rumble(1, 1, durationMs);
		if (gamepad2 != null)
			gamepad2.rumble(1, 1, durationMs);
	}

}
