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
import org.firstinspires.ftc.teamcode.util.Alliance;

import com.pedropathing.geometry.Pose;

public class Automations {
	private HardwareMap hardwareMap;
	private Alliance alliance;
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

	private Pose pose;
	private Artifact.Pattern pattern;
	private boolean intakeEnabled;

	public enum StorageState {
		WAITING,
		STORING,
		RELEASING
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance) {
		this(hardwareMap, alliance, false);
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance, boolean DEBUG) {
		this.hardwareMap = hardwareMap;
		this.alliance = alliance;
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

	public void setAlliance(Alliance alliance) {
		this.alliance = alliance;
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
		} else {
			Pose goalPose = alliance.getGoalPose();
			Pose poseDifference = goalPose.minus(pose);

			// Converting to normal coordinate system where
			// 0 = up, increases clockwise; In radians
			double robotAngle = (0.5 * Math.PI) - pose.getHeading();
			robotAngle = robotAngle % (2 * Math.PI);
			double targetAngle = Math.atan2(poseDifference.getX(), poseDifference.getY());

			double angleDifference = targetAngle - robotAngle;
			double normalizedAngle = angleDifference - (Math.ceil((angleDifference + Math.PI) / (2 * Math.PI)) - 1)
					* 2 * Math.PI;

			turret.setRotation(Math.toDegrees(normalizedAngle) * Turret.rotation_per_deg);
		}
	}

	// Should be called every loop. Pose is used to estimate
	// turret direction
	public void updatePose(Pose pose) {
		this.pose = pose;
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

	public Alliance getAlliance() {
		return alliance;
	}

	public StorageState getStorageState() {
		return storageState;
	}

	public Artifact.Pattern getPattern() {
		return pattern;
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
