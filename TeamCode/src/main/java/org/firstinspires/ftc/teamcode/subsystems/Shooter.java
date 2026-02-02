package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pidf;
import org.firstinspires.ftc.teamcode.util.InterpolatedLUT;

@Configurable
@Config
public class Shooter {
	public static double velocityConstant = -150;
	public static double velocityTolerance = 50;
	public static double shootingAreaTolerance = 12.7279220614;
	public static double defaultVelocity = 2200;
	public static double PIDF_P = 0.03;
	public static double PIDF_I = 0;
	public static double PIDF_D = 0;
	public static double PIDF_F = 0.0004;
	public static boolean PIDF_update = false;

	public static double min_pitch = 0.485;
	public static double max_pitch = 0.73;
	public static double pitch_increment = 0.01;

	public static double goal_height = 46.25;
	public static double robot_height = 9.175;
	public static double goal_angle = -30;

	private final double GRAVITY = DistanceUnit.INCH.fromMeters(9.80665);

	public double desiredVelocity = 2200;

	private Pidf pidfController;
	private boolean enabled;
	private double power;

	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;

	private Servo shooterPitchServo;

	private static double pitch = min_pitch;

	// Angle to servo value
	private static final InterpolatedLUT<Double> hoodAngleLUT = new InterpolatedLUT<Double>();

	static {
		hoodAngleLUT.add(
			hoodAngleLUT.new Entry(61, 0.15),
			hoodAngleLUT.new Entry(59, 0.21),
			hoodAngleLUT.new Entry(55, 0.27),
			hoodAngleLUT.new Entry(54, 0.3),
			hoodAngleLUT.new Entry(52, 0.35),
			hoodAngleLUT.new Entry(51, 0.4),
			hoodAngleLUT.new Entry(50, 0.45)
		);
	}

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
		shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
		shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

		shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		pidfController = new Pidf(PIDF_P, PIDF_I, PIDF_D, PIDF_F);

		shooterPitchServo = hardwareMap.get(Servo.class, "turretPitchServo");
		shooterPitchServo.setDirection(Servo.Direction.FORWARD);
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

	public void setPower(double pow) {
		pow = Range.clip(pow, -1, 1);
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
		power = pow;
	}

	public static double convertProjectileToShooterVelocity(double projectileVelocity) {
		// TODO: Create equation based on testing
		return projectileVelocity;
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose) {
		updateShooterTarget(robotPose, targetPose, new Vector());
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose, Vector robotVelocity) {
		double horizontalDistance = robotPose.distanceFrom(targetPose);
		double verticalDistance = goal_height - robot_height;

		double angle = Math.atan((2 * verticalDistance) / horizontalDistance - Math.tan(goal_angle));
		double v0 = Math.sqrt((GRAVITY * Math.pow(horizontalDistance, 2))
				/ (2 * Math.pow(Math.cos(angle), 2) * (horizontalDistance * Math.tan(angle) - verticalDistance)));

		double velocityAngleDifference = robotVelocity.getTheta()
				- targetPose.minus(robotPose).getAsVector().getTheta();
		double radialVelocity = Math.cos(velocityAngleDifference) * robotVelocity.getMagnitude();
		double tangentialVelocity = Math.sin(velocityAngleDifference) * robotVelocity.getMagnitude();

		double time = horizontalDistance / (v0 * Math.cos(angle));

		double vxNew = Math.hypot((horizontalDistance / time) + radialVelocity, tangentialVelocity);
		double vyNew = v0 * Math.sin(angle);

		double newAngle = Math.atan(vyNew / vxNew);
		double newHorizontalDistance = vxNew * time;
		double newV0 = Math.sqrt((GRAVITY * Math.pow(newHorizontalDistance, 2))
				/ (2 * Math.pow(Math.cos(newAngle), 2) * (newHorizontalDistance * Math.tan(newAngle) - verticalDistance)));

		desiredVelocity = convertProjectileToShooterVelocity(newV0);
		setPitchAngle(newAngle);
	}

	public void updateVelocityPid() {
		if (!enabled)
			return;
		if (PIDF_update) {
			pidfController.setKp(PIDF_P);
			pidfController.setKi(PIDF_I);
			pidfController.setKd(PIDF_D);
			pidfController.setKv(PIDF_F);
		}

		double pidOutput = pidfController.calculate(desiredVelocity, getVelocity());
		setPower(Range.clip(pidOutput, 0, 1));
	}

	public void showPidTelemetry(TelemetryManager telemetry) {
		telemetry.addData("Shooter power", power);
		pidfController.showTelemetry(telemetry);
	}

	public boolean atDesiredVelocity() {
		return getVelocity() >= desiredVelocity;
	}

	public double getVelocity() {
		return shooterMotorLeft.getVelocity();
	}

	public static double getPitch() {
		return Shooter.pitch;
	}

	public void pitchTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setPitch(pitch + (vector * pitch_increment));
	}

	public void setPitchAngle(double angle) {
		Double hoodPosition = hoodAngleLUT.get(angle);
		if (hoodPosition == null) {
			return;
		}
		setPitch(hoodPosition);
	}

	public void setPitch(double pitchTarget) {
		pitchTarget = Range.clip(pitchTarget, min_pitch, max_pitch);

		shooterPitchServo.setPosition(pitchTarget);

		Shooter.pitch = pitchTarget;
	}
}
