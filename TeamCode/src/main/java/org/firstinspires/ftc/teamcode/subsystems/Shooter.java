package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pidfs;
import org.firstinspires.ftc.teamcode.util.InterpolatedLUT;

@Configurable
@Config
public class Shooter {
	public static double velocityConstant = 0;
	public static double velocityTolerance = 50;
	public static double shootingAreaTolerance = 12.7279220614;
	public static double defaultVelocity = 1680;
	public static double defaultAngle = 55.9859280971;
	public static double velocityDrop = 60;
	public static int velocityDropTimeMs = 90;
	public static double minPower = 0.0;
	public static double voltageCompTarget = 12;
	public static double PIDF_P = 0.004;
	public static double PIDF_I = 0;
	public static double PIDF_D = 0;
	public static double PIDF_F = 0.00035;
	public static double PIDF_FS = 0.06;
	public static boolean PIDF_update = false;

	public static double min_pitch = 0;
	public static double max_pitch = 0.65;
	public static double pitch_increment = 0.01;

	public static double goal_height = 20;
	public static double robot_height = 9.175;
	public static double goal_angle = -45;

	private final double GRAVITY = DistanceUnit.INCH.fromMeters(9.80665);

	public double desiredVelocity = 0;

	private Pidfs pidfController;
	private boolean enabled;
	private double power;

	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;
	private VoltageSensor voltageSensor;

	private Servo shooterPitchServo;

	private static double pitch = min_pitch;
	private static double pitchDegrees = 0;

	// Projectile velocity (v0) to shooter velocity
	private static final InterpolatedLUT<Double> velocityLUT = new InterpolatedLUT<Double>();
	// Angle to servo value
	private static final InterpolatedLUT<Double> hoodAngleLUT = new InterpolatedLUT<Double>();

	static {
		/*
		 * Projectile velocity obtained by finding working flywheel speed at a point
		 * and reversing equation to find expected linear velocity at that point.
		 * Original data can be found in Notes.md
		 */
		velocityLUT.add(
			velocityLUT.new Entry(145.779222955, 980d),
			velocityLUT.new Entry(176.089150096, 1280d),
			velocityLUT.new Entry(187.574188827, 1400d),
			velocityLUT.new Entry(109.742384956, 600d),
			velocityLUT.new Entry(167.282727863, 1260d),
			velocityLUT.new Entry(210.5801596, 1640d),
			velocityLUT.new Entry(309.496549403, 2100d),
			velocityLUT.new Entry(326.083917735, 2200d),
			velocityLUT.new Entry(313.747917615, 2120d),
			velocityLUT.new Entry(342.007009653, 2340d),
			velocityLUT.new Entry(260.751045421, 1740d),
			velocityLUT.new Entry(241.522208123, 1600d),
			velocityLUT.new Entry(219.013459166, 1480d),
			velocityLUT.new Entry(228.505232468, 1520d),
			velocityLUT.new Entry(203.158129057, 1380d)
		);

		hoodAngleLUT.add(
			hoodAngleLUT.new Entry(90-36, 0.7),
			hoodAngleLUT.new Entry(90-39, 0.6),
			hoodAngleLUT.new Entry(90-42, 0.5),
			hoodAngleLUT.new Entry(90-45, 0.4),
			hoodAngleLUT.new Entry(90-49, 0.3),
			hoodAngleLUT.new Entry(90-52, 0.2),
			hoodAngleLUT.new Entry(90-56, 0.08),
			hoodAngleLUT.new Entry(90-58, 0.0)
		);
	}

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
		shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
		shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

		shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		shooterMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

		pidfController = new Pidfs(PIDF_P, PIDF_I, PIDF_D, PIDF_F, PIDF_FS);

		shooterPitchServo = hardwareMap.get(Servo.class, "turretPitchServo");
		shooterPitchServo.setDirection(Servo.Direction.FORWARD);

		voltageSensor = hardwareMap.voltageSensor.iterator().next();
	}

	public void enable(boolean enabled) {
		if (enabled) {
			setPower(power);
		} else {
			shooterMotorLeft.setPower(0);
			shooterMotorRight.setPower(0);
		}
		this.enabled = enabled;
	}

	public boolean isEnabled() {
		return enabled;
	}

	public void setPower(double pow) {
		pow = Range.clip(pow, -1, 1);
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
		power = pow;
	}

	public double getPower() {
		return power;
	}

	public static double convertProjectileToShooterVelocity(double projectileVelocity) {
		return velocityLUT.get(projectileVelocity);
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose) {
		updateShooterTarget(robotPose, targetPose, new Vector());
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose, Vector robotVelocity) {
		desiredVelocity = defaultVelocity;
		setPitchAngle(Math.toDegrees(defaultAngle));
	}

	public void updateVelocityPid() {
		if (!enabled)
			return;
		if (PIDF_update) {
			pidfController.setKp(PIDF_P);
			pidfController.setKi(PIDF_I);
			pidfController.setKd(PIDF_D);
			pidfController.setKv(PIDF_F);
			pidfController.setKs(PIDF_FS);
		}

		double pidOutput = pidfController.calculate(desiredVelocity, getVelocity());
		if (pidOutput == PIDF_FS) {
			pidOutput = 0;
		}
		setPower(Range.clip(pidOutput * (voltageSensor.getVoltage() / voltageCompTarget), minPower, 1));
	}

	public void showPidTelemetry(TelemetryManager telemetry) {
		telemetry.addData("Shooter power", power);
		pidfController.showTelemetry(telemetry);
	}

	public boolean atDesiredVelocity() {
		return getVelocity() >= desiredVelocity;
	}

	public double getVelocity() {
		return shooterMotorRight.getVelocity();
	}

	public static double getPitch() {
		return Shooter.pitch;
	}

	public static double getPitchDegrees() {
		return pitchDegrees;
	}

	public void pitchTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setPitch(pitch + (vector * pitch_increment));
	}

	public void setPitchAngle(double angle) {
		Double hoodPosition = hoodAngleLUT.get(angle);
		if (hoodPosition == null || hoodPosition.isNaN()) {
			return;
		}
		setPitch(hoodPosition);
		pitchDegrees = angle;
	}

	public void setPitch(double pitchTarget) {
		pitchTarget = Range.clip(pitchTarget, min_pitch, max_pitch);

		shooterPitchServo.setPosition(pitchTarget);

		Shooter.pitch = pitchTarget;
	}
}
