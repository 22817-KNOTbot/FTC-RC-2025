package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Brakes;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight.AlignmentDirection;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pid;

import com.pedropathing.math.Vector;
import com.pedropathing.geometry.Pose;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.knotbot.practiceapp.RobotEvent;

@Config
@Configurable
public class Automations {
	public static boolean useVision = true;

	private Alliance alliance;
	private boolean DEBUG;

	private Intake intake;
	private Transfer transfer;
	private Turret turret;
	private Shooter shooter;
	private Limelight limelight;
	private Brakes brakes;
	private Light light;

	private Gamepad gamepad1;
	private Gamepad gamepad2;

	private Pose pose;
	private Vector velocity;

	private Pid turretPid;

	private State state = State.IDLE;
	private boolean shootInit;
	private boolean ignoreVelocity;
	private boolean intakeEjecting;
	private boolean intakeLastLoaded;

	public enum State {
		IDLE,
		INTAKING,
		WAITING_TO_SHOOT,
		SHOOTING,
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
		this(hardwareMap, alliance, resetEncoders, false);
	}

	public Automations(HardwareMap hardwareMap, Alliance alliance, boolean resetEncoders, boolean DEBUG) {
		this.alliance = alliance;
		this.DEBUG = DEBUG;
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		turret = new Turret(hardwareMap);
		shooter = new Shooter(hardwareMap);

		limelight = new Limelight(hardwareMap, alliance.getGoalAprilTagId(), DEBUG);

		brakes = new Brakes(hardwareMap);
		light = new Light(hardwareMap);

		turretPid = new Pid(Turret.Kp, Turret.Ki, Turret.Kd);
	}
	
	public void setAlliance(Alliance alliance) {
		this.alliance = alliance;
		limelight.setTargetAprilTagId(alliance.getGoalAprilTagId());
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	public void showTelemetry(TelemetryManager telemetry) {
		telemetry.addData("In shooting zone", inShootingArea());
		telemetry.addData("Shooter Desired Velocity", shooter.desiredVelocity);
		telemetry.addData("Distance", pose.distanceFrom(alliance.getGoalPose()));
		telemetry.addData("Turret target", Turret.getTargetRotation());
		telemetry.addData("Turret calculated angle", Turret.getTargetRotation() / Turret.rotation_per_deg);
		telemetry.addData("Shooter pitch deg", Shooter.getPitchDegrees());
		telemetry.addData("Using vision", useVision);
		telemetry.addData("Vision Alignment Direction", limelight.getAlignmentDirection());
		telemetry.addData("Intake loaded", intake.getLoaded());
		telemetry.addData("Transfer loaded", transfer.getLoaded());
	}

	public void abort() {
		intake.enable(false);
		intakeEjecting = false;
		transfer.enable(false);
		state = State.IDLE;
	}

	// Code that should be run on start but not during init
	public void start() {
		// For future use
	}

	// Should be called every loop. Handles various things
	// that need to be called repeatedly
	public void automationLoop() {
		shooter.updateVelocityPid();
		turret.update();

		switch (state) {
			case IDLE:
				break;
			case INTAKING:
				if (transfer.getLoaded()) {
					transfer.enable(false);
				}
				if (intake.intakeUpdate() && transfer.getLoaded()) {
					vibrateControllers();
					intakeEnable(false);
					intake.intakeReset();
					intakeLastLoaded = true;
					state = State.IDLE;
				} else {
					intakeLastLoaded = false;
				}
				break;
			case WAITING_TO_SHOOT:
				if (shooter.atDesiredVelocity() || ignoreVelocity) {
					if (getShootingArea() == ShootingZone.LOWER) {
						intake.enableSlow(true);
						transfer.enableSlowFar(true);
					} else {
						intake.enable(true);
						transfer.enable(true);
					}
					shooter.enable(true);
					shootInit = false;
					state = State.SHOOTING;
				}
				break;
			case SHOOTING:
				if (!shootInit) {
					RobotEvent.addScore(3, "Artifact");
					RobotEvent.addScore(3, "Artifact");
					RobotEvent.addScore(3, "Artifact");
					shootInit = true;
				}
				transfer.transferUpdate();
				intakeLastLoaded = false;
				// if (transfer.isFinishedTransferring()) {
				// 	vibrateControllers();
				// 	setShooting(false);
				// }
				break;
		}

		if (brakes.isBrakesEngaged()) {
			light.setRed();
		} else if (transfer.getLoaded() && intakeLastLoaded) {
			light.setBlue();
		} else {
			light.setGreen();
		}
	}

	// Should only be called once as the opmode ends
	public void end() {
		// For future use
	}

	// Should be called to update the turret
	public void updateTurret() {
		AlignmentDirection direction = limelight.getAlignmentDirection();
		if (!direction.directionKnown || !useVision) {
			turretPid.reset();
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
			double bearing = direction.bearing;
			
			if (getShootingArea() == ShootingZone.LOWER) {
				if (alliance instanceof RedAlliance) {
					bearing += Turret.vision_far_offset_deg;
				} else if (alliance instanceof BlueAlliance) {
					bearing -= Turret.vision_far_offset_deg;
				}
			}
			if (DEBUG) {
				turretPid.setKp(Turret.Kp);
				turretPid.setKi(Turret.Ki);
				turretPid.setKd(Turret.Kd);
			}
			double difference = turretPid.calculate(bearing, 0);
			double targetRotation = Turret.getTargetRotation() + difference;
			turret.setRotation(targetRotation);
		}
	}

	// Should be called to update the shooter velocity
	public void updateShooter() {
		if (inShootingArea()) {
			setShooterEnabled(true);
			shooter.updateShooterTarget(pose, alliance.getGoalShooterPose());
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

	/**
	 * Enables or disables the intake depending on the paramter.
	 * Note that this changes the state to INTAKING or IDLE and should
	 * not be used if this is not desired.
	 * @param enable Whether to enable the or disable the intake
	 */
	public void intakeEnable(boolean enable) {
		intake.enable(enable);
		transfer.enableSlow(enable);
		if (enable) {
			state = State.INTAKING;
		} else {
			state = State.IDLE;
		}
	}

	public void intakeToggle() {
		intakeEnable(!intake.isEnabled());
	}

	public void intakeEject() {
		intake.enableReversed(true);
		transfer.enableReversed(true);
		intakeEjecting = true;
	}

	public void intakeEjectStop() {
		intake.enable(false);
		transfer.enable(false);
		intakeEjecting = false;
	}

	public void startShooting() {
		setShooting(true);
	}

	public void setShooting(boolean shooting) {
		if (shooting) {
			transfer.transferReset();
			state = State.WAITING_TO_SHOOT;
		} else {
			intake.enable(false);
			transfer.enable(false);
			shooter.enable(false);
			state = State.IDLE;
		}
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
		shooter.pitchTurret(vector);
	}

	public void engageBrakes(boolean engage) {
		brakes.engageBrakes(engage);
	}

	/*
	 * Getter methods
	 */

	public State getState() {
		return state;
	}

	public boolean isFinishedShooting() {
		return state == State.SHOOTING && transfer.isFinishedTransferring();
	}

	public Alliance getAlliance() {
		return alliance;
	}

	public boolean getShooterEnabled() {
		return shooter.isEnabled();
	}

	public double getShooterVelocity() {
		return shooter.getVelocity();
	}

	public double getShooterDesiredVelocity() {
		return shooter.desiredVelocity;
	}
	
	public boolean getIntakeEjecting() {
		return intakeEjecting;
	}

	public void setIgnoreVelocity(boolean ignoreVelocity) {
		this.ignoreVelocity = ignoreVelocity;
	}

	public void setUseVision(boolean useVision) {
		this.useVision = useVision;
	}

	public boolean getVisionAlignmentKnown() {
		return limelight.getAlignmentDirection().directionKnown;
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
		} else if ((futurePose.getY() + (Math.abs(futurePose.getX() - 72)) <= 24 + Shooter.shootingAreaTolerance && futurePose.getY() >= 0) ||
				(pose.getY() + (Math.abs(pose.getX() - 72)) <= 24 + Shooter.shootingAreaTolerance && pose.getY() >= 0)) {
			return ShootingZone.LOWER;
		} else {
			return ShootingZone.NONE;
		}
	}

	public void setShooterEnabled(boolean enabled) {
		shooter.enable(enabled);
	}

	public void setShooterVelocity(double velocity) {
		shooter.desiredVelocity = velocity;
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
