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

import com.pedropathing.math.Vector;
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
	private Vector velocity;

	private Artifact.Pattern pattern;
	private boolean intakeEnabled;
	private boolean shooterEnabled;

	public enum StorageState {
		WAITING,
		TURNING,
		TRANSFERRING
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
		storage = new Storage(hardwareMap, false);
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
		intake.enable(false);
		storage.abort();
		shooter.enable(false);
	}

	// Code that should be run on start but not during init
	public void start() {
		setShooterEnabled(true);
	}

	// Should be called every loop. Handles various things
	// that need to be called repeatedly
	public void automationLoop() {
		vision.updateMotifPattern();
		pattern = vision.getLastMotifPattern();
		if (storageState == StorageState.WAITING) {
			storage.intake();
		} else if (storageState == StorageState.TURNING && shooter.getVelocity() >= Shooter.shooterVelocity 
				&& timer.time() > 0.1){ // may change time later
			shootActiveArtifact();
			timer.reset();
		} else if (storageState == StorageState.TRANSFERRING) {
			storage.transferUpdate();
			if (storage.transferAll && storage.transferState == Storage.TransferState.IDLE) {
				intake.enable(true);
				if (storage.getActiveArtifact() != null) {
					shootActiveArtifact();
					timer.reset();
				} else {
					shooter.enable(false);
					storage.finishTransfer();
					intake.enable(false);
					storageState = StorageState.WAITING;
				}
			}
		}
	}

	// Should only be called once as the opmode ends
	public void end() {
		vision.close();
	}

	// Should be called every loop
	public void updateTurret() {
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

	// Should be called every loop. Pose is used to estimate
	// turret direction
	public void updatePose(Pose pose) {
		this.pose = pose;
	}
	public void updateVelocity(Vector velocity) {
		this.velocity = velocity;
	}

	public void intakeToggle() {
		intake.enable(!intakeEnabled);
		intakeEnabled = !intakeEnabled;
	}

	public Storage.TurnDirection prepareArtifact(Artifact.Colour colour) {
		Storage.TurnDirection turnDirection = storage.turnToArtifact(colour, true);
		timer.reset();
		storageState = StorageState.TURNING;
		return turnDirection;
	}

	// Returns true if already prepared and has been shot
	public boolean prepareOrShootArtifact(Artifact.Colour colour) {
		Storage.TurnDirection turnDirection = storage.turnToArtifact(colour, true);
		if (turnDirection == Storage.TurnDirection.AVAILABLE) {
			shootActiveArtifact();
			return true;
		} else if (turnDirection == Storage.TurnDirection.NONE) {
			vibrateControllers();
		} else {
			timer.reset();
			storageState = StorageState.TURNING;
		}
		return false;
	}

	public void storageTurnCW() {
		storage.storageTurnCW();
	}

	public void storageTurnCCW() {
		storage.storageTurnCCW();
	}

	public void shootActiveArtifact() {
		shooter.enable(true);
		storage.transfer();
		storageState = StorageState.TRANSFERRING;
	}

	public boolean colourSensorResponding() {
		return storage.colourSensorResponding();
	}

	public void rotateTurret(double vector) {
		turret.rotateTurret(vector);
	}

	public void pitchTurret(double vector) {
		turret.pitchTurret(vector);
	}

	/*
	 * Getter methods
	 */

	public Alliance getAlliance() {
		return alliance;
	}

	public Artifact.Pattern getPattern() {
		return pattern;
	}

	public StorageState getStorageState() {
		return storageState;
	}

	public boolean getShooterEnabled() {
		return shooterEnabled;
	}

	public Artifact.Pattern getArtifactPattern() {
		return pattern;
	}

	/*
	 * Misc. util methods
	 */

	public boolean inShootingArea() {
		Pose futurePose = pose.plus(new Pose(velocity.getXComponent(), velocity.getYComponent()));
		return ((futurePose.getY() - 72) >= Math.abs(futurePose.getX() - 72) ||
				(pose.getY() - 72) >= Math.abs(pose.getX() - 72)) ||
				((futurePose.getY() + (Math.abs(futurePose.getX() - 72)) <= 24) ||
				(pose.getY() + (Math.abs(pose.getX() - 72)) <= 24));
	}

	public void setShooterEnabled(boolean enabled) {
		shooter.enable(enabled);
		shooterEnabled = enabled;
	}

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
