package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoAlign.AlignmentDirection;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.pedropathing.math.Vector;
import com.pedropathing.geometry.Pose;

public class Automations {
	private HardwareMap hardwareMap;
	private Alliance alliance;
	private boolean DEBUG;
	private StorageState storageState;
	private ElapsedTime stateTimer;
	private ElapsedTime ejectTimer;
	private TelemetryManager telemetryManager;

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
	private boolean intakeEjecting;
	private boolean intakeTimedEjecting;
	private boolean shooterEnabled;
	private boolean ignoreVelocity;
	private boolean transferAll;
	private boolean visionOverridingTurret;
	private boolean visionOverridedTurret;
	private boolean useVision = true;

	public enum StorageState {
		WAITING,
		INTAKING,
		TURNING_TRANSFER,
		TRANSFERRING
	}

	public enum ShootingZone {
		UPPER,
		LOWER,
		NONE
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance) {
		this(hardwareMap, alliance, false);
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance, boolean resetEncoders) {
		this(hardwareMap, alliance, false, false);
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance, boolean resetEncoders, boolean DEBUG) {
		this.hardwareMap = hardwareMap;
		this.alliance = alliance;
		this.DEBUG = DEBUG;
		this.storageState = StorageState.WAITING;
		this.stateTimer = new ElapsedTime();
		this.ejectTimer = new ElapsedTime();
		intake = new Intake(hardwareMap);
		storage = new Storage(hardwareMap, resetEncoders);
		turret = new Turret(hardwareMap);
		shooter = new Shooter(hardwareMap);

		Vision.DEBUG = DEBUG;
		vision = new Vision(hardwareMap, null);
		vision.setMotifPrioritySide(alliance.getObeliskSidePriority());
		vision.setTargetAprilTagId(alliance.getGoalAprilTagId());
	}
	
	public void setAlliance(Alliance alliance) {
		this.alliance = alliance;
		vision.setMotifPrioritySide(alliance.getObeliskSidePriority());
		vision.setTargetAprilTagId(alliance.getGoalAprilTagId());
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	public void showTelemetry(TelemetryManager telemetry) {
		telemetry.addData("In shooting zone", inShootingArea());
		telemetry.addData("Shooter Desired Velocity", shooter.desiredVelocity);
		telemetry.addData("Distance", pose.distanceFrom(alliance.getGoalPose()));
		telemetry.addData("Intake timed ejecting", intakeTimedEjecting);
		telemetry.addData("Using vision", useVision);
		telemetry.addData("Vision Alignment Direction", vision.getAlignmentDirection());
		storage.showTelemetry(telemetry);
		telemetryManager = telemetry;
		// vision.showTelemetry(telemetry);
	}

	public void abort() {
		storageState = StorageState.WAITING;
		intakeTimedEjecting = false;
		intake.enable(false);
		storage.abort();
		setVisionOverrideEnabled(false);
	}

	// Code that should be run on start but not during init
	public void start() {
		// setShooterEnabled(true);
		storage.storageMotorEnable(true);
		storage.start();
	}

	// Should be called every loop. Handles various things
	// that need to be called repeatedly
	public void automationLoop() {
		updateMotifPattern();

		shooter.updateVelocityPid();
		if (storageState == StorageState.WAITING && intakeEnabled) {
			storageState = StorageState.INTAKING;
		} else if (storageState == StorageState.INTAKING) {
			if (storage.intakeUpdate()) {
				vibrateControllers();
				intake.enableReversed(true);
				intakeEnabled = false;
				intakeTimedEjecting = true;
				ejectTimer.reset();
				storageState = StorageState.WAITING;
			}
		} else if (storageState == StorageState.TURNING_TRANSFER && !storage.isMotorBusy()) {
			shootActiveArtifact();
			storage.transferStart();
			stateTimer.reset();
		} else if (storageState == StorageState.TRANSFERRING) {
			// Pause transfer updates while waiting to reach velocity
			storage.transferUpdate((isShooterAtVelocity() || ignoreVelocity));
			if (storage.getTransferState() == Storage.TransferState.RESET) {
				if (transferAll && Storage.getActiveArtifact() != null) {
					if (isShooterAtVelocity()) {
						shootActiveArtifact();
						stateTimer.reset();
					}
				} else {
					storage.transferFinish();
					setVisionOverrideEnabled(false);
					intake.enable(false);
					storageState = StorageState.WAITING;
				}
			}
		}

		if (intakeTimedEjecting && ejectTimer.time() > 0.5) {
			intakeEnable(false);
			intakeTimedEjecting = false;
		}
	}

	// Should only be called once as the opmode ends
	public void end() {
		vision.close();
	}

	// Should be called to update the turret
	public void updateTurret() {
		if (!visionOverridingTurret) {
			Pose goalPose = alliance.getGoalPose();
			Vector turretOffset = pose.getHeadingAsUnitVector().times(4);
			Pose turretPose = pose.plus(new Pose(turretOffset.getXComponent(), turretOffset.getYComponent()));
			Pose poseDifference = goalPose.minus(turretPose);

			// Converting to normal coordinate system where
			// 0 = up, increases clockwise; In radians
			double robotAngle = (0.5 * Math.PI) - pose.getHeading();
			robotAngle = robotAngle % (2 * Math.PI);
			double targetAngle = Math.atan2(poseDifference.getX(), poseDifference.getY());

			double angleDifference = targetAngle - robotAngle;
			double normalizedAngle = angleDifference - (Math.ceil((angleDifference + Math.PI) / (2 * Math.PI)) - 1)
					* 2 * Math.PI;

			double targetRotation = Turret.BASE_ROTATION + Math.toDegrees(normalizedAngle) * Turret.rotation_per_deg;
			turret.setRotation(targetRotation);
		} else {
			if (!useVision) {
				setVisionOverrideEnabled(false);
				updateTurret();
				return;
			}
			if (visionOverridedTurret)
				return;

			AlignmentDirection direction = vision.getAlignmentDirection();
			if (!direction.directionKnown) {
				setVisionOverrideEnabled(false);
				return;
			}
			double bearing = direction.bearing;

			double targetRotation = Turret.getRotation() - (bearing * Turret.rotation_per_deg) + Turret.vision_offset;
			turret.setRotation(targetRotation);
			visionOverridedTurret = true;
		}
	}

	// Should be called to update the shooter velocity
	public void updateShooter() {
		shooter.updateVelocityTarget(pose.distanceFrom(alliance.getGoalShooterPose()));
		if (inShootingArea()) {
			setShooterEnabled(true);
			switch (getShootingArea()) {
				case UPPER:
					turret.setPitch(Turret.max_pitch);
					break;
				case LOWER:
					turret.setPitch(Turret.min_pitch);
				default:
					break;
			}
		} else {
			setShooterEnabled(false);
		}
	}

	// Should be called every loop. Pose is used to estimate
	// turret direction
	public void updatePose(Pose pose) {
		this.pose = pose;
	}

	public void updateVelocity(Vector velocity) {
		this.velocity = velocity;
	}

	public void updateMotifPattern() {
		vision.updateMotifPattern();
		pattern = vision.getLastMotifPattern();
	}

	public void intakeEnable(boolean enable) {
		intake.enable(enable);
		intakeEnabled = enable;
		intakeEjecting = false;
		intakeTimedEjecting = false;
	}

	public void intakeToggle() {
		intakeEnable(!intakeEnabled);
	}

	public void intakeEnableActions(boolean enable) {
		intakeEnable(enable);
	}

	public void intakeEject() {
		intake.enableReversed(true);
		intakeEnabled = false;
		intakeEjecting = true;
		intakeTimedEjecting = false;
	}

	public void intakeEjectStop() {
		intakeEnable(false);
	}

	public Storage.TurnDirection prepareArtifact(Artifact.Colour colour) {
		Storage.TurnDirection turnDirection = storage.turnToArtifact(colour);
		stateTimer.reset();
		storageState = StorageState.TURNING_TRANSFER;
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
			// storage.gateUp();
			stateTimer.reset();
			storageState = StorageState.TURNING_TRANSFER;
		}
		return false;
	}

	public Storage.TurnDirection prepareOrShootArtifactSequence(Artifact.Colour[] sequence) {
		Storage.TurnDirection turnDirection = storage.turnToArtifactSequence(sequence);
		if (turnDirection == Storage.TurnDirection.AVAILABLE) {
			shootActiveArtifact();
		} else if (turnDirection == Storage.TurnDirection.NONE) {
			vibrateControllers();
		} else {
			stateTimer.reset();
			storageState = StorageState.TURNING_TRANSFER;
		}
		return turnDirection;
	}

	public boolean prepareOrShootAnyArtifact() {
		Storage.TurnDirection turnDirection = storage.turnToAnyArtifact();
		if (turnDirection == Storage.TurnDirection.AVAILABLE) {
			shootActiveArtifact();
			return true;
		} else if (turnDirection == Storage.TurnDirection.NONE) {
			vibrateControllers();
		} else {
			stateTimer.reset();
			storageState = StorageState.TURNING_TRANSFER;
		}
		return false;
	}

	public void storageTurnCW() {
		storage.storageTurnCW();
	}

	public void storageTurnCCW() {
		storage.storageTurnCCW();
	}

	public void setRapidFire(boolean enabled) {
		transferAll = enabled;
		if (enabled) {
			setTransferMode(Storage.TransferMode.FULL_SPIN);
		} else {
			setTransferMode(Storage.TransferMode.NORMAL);
		}
	}

	public void shootActiveArtifact(boolean force) {
		shooter.enable(true);
		setVisionOverrideEnabled(true);
		storageState = StorageState.TRANSFERRING;
	}

	public void shootActiveArtifact() {
		shootActiveArtifact(false);
	}

	public boolean colourSensorActiveResponding() {
		return storage.colourSensorActiveResponding();
	}

	public boolean colourSensorBackLeftResponding() {
		return storage.colourSensorBackLeftResponding();
	}

	public boolean colourSensorBackRightResponding() {
		return storage.colourSensorBackRightResponding();
	}

	public boolean colourSensorsResponding() {
		return storage.colourSensorsResponding();
	}

	public void setTurretRotationDegrees(double positionDegrees) {
		turret.setRotation(Turret.BASE_ROTATION + Turret.rotation_per_deg * positionDegrees);
	}

	public void setTurretRotation(double position) {
		turret.setRotation(position);
	}

	public void rotateTurret(double vector) {
		turret.rotateTurret(vector);
	}

	public void pitchTurret(double vector) {
		turret.pitchTurret(vector);
	}

	public void clearStorageMemory() {
		storage.clearStorageMemory();
	}

	public void setVisionOverrideEnabled(boolean enable) {
		visionOverridingTurret = enable;
		visionOverridedTurret = false;
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

	public List<Colour> getArtifactsStored() {
		return storage.getArtifactsStored();
	}

	public void setArtifactsStored(Colour[] colours) {
		Storage.setArtifactsStored(colours);
	}

	public StorageState getStorageState() {
		return storageState;
	}

	public Storage.IntakeState getIntakeState() {
		return storage.getIntakeState();
	}

	public Storage.TransferState getTransferState() {
		return storage.getTransferState();
	}

	public void setTransferMode(Storage.TransferMode transferMode) {
		storage.setTransferMode(transferMode);
	}

	public boolean getShooterEnabled() {
		return shooterEnabled;
	}

	public double getShooterVelocity() {
		return shooter.getVelocity();
	}

	public double getShooterDesiredVelocity() {
		return shooter.desiredVelocity;
	}

	public double getShooterVelocityTargetAtDistance(double distance) {
		return shooter.getVelocityTarget(distance);
	}

	public Artifact.Pattern getArtifactPattern() {
		return pattern;
	}

	public boolean getIntakeEjecting() {
		return intakeEjecting;
	}

	public boolean getRapidFire() {
		return transferAll;
	}

	public void setIgnoreVelocity(boolean ignoreVelocity) {
		this.ignoreVelocity = ignoreVelocity;
	}

	public void setUseVision(boolean useVision) {
		this.useVision = useVision;
	}

	public boolean getVisionOverridingTurret() {
		return visionOverridingTurret;
	}

	public boolean getVisionAlignmentKnown() {
		return vision.getAlignmentDirection().directionKnown;
	}

	/*
	 * Misc. util methods
	 */

	public boolean inShootingArea() {
		return getShootingArea() != ShootingZone.NONE;
	}

	public ShootingZone getShootingArea() {
		if (velocity == null) {
			velocity = new Vector();
		}
		Pose futurePose = pose.plus(new Pose(velocity.getXComponent(), velocity.getYComponent()));
		if ((futurePose.getY() - 72) + Shooter.shootingAreaTolerance >= Math.abs(futurePose.getX() - 72) ||
				(pose.getY() - 72) + Shooter.shootingAreaTolerance >= Math.abs(pose.getX() - 72)) {
			return ShootingZone.UPPER;
		} else if ((futurePose.getY() + (Math.abs(futurePose.getX() - 72)) <= 24 + Shooter.shootingAreaTolerance) ||
				(pose.getY() + (Math.abs(pose.getX() - 72)) <= 24 + Shooter.shootingAreaTolerance)) {
			return ShootingZone.LOWER;
		} else {
			return ShootingZone.NONE;
		}
	}

	public void setShooterEnabled(boolean enabled) {
		shooter.enable(enabled);
		shooterEnabled = enabled;
	}

	public void setShooterVelocity(double velocity) {
		shooter.desiredVelocity = velocity;
	}

	public boolean isShooterAtVelocity() {
		return Math.abs(shooter.getVelocity() - shooter.desiredVelocity) < Shooter.velocityTolerance;
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

	public void vibrateControllersBlips(int blips) {
		if (gamepad1 != null)
			gamepad1.rumbleBlips(blips);
		if (gamepad2 != null)
			gamepad2.rumbleBlips(blips);
	}
}
