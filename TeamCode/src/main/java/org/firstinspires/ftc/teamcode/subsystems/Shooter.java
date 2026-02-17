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
	public static double velocityConstant = 0;
	public static double velocityTolerance = 50;
	public static double shootingAreaTolerance = 12.7279220614;
	public static double defaultVelocity = 1680;
	public static double defaultAngle = 55.9859280971;
	public static double PIDF_P = 0.07;
	public static double PIDF_I = 0;
	public static double PIDF_D = 0;
	public static double PIDF_F = 0.0004;
	public static boolean PIDF_update = false;

	public static double min_pitch = 0.15;
	public static double max_pitch = 0.45;
	public static double pitch_increment = 0.01;

	public static double goal_height = 46.25;
	public static double robot_height = 9.175;
	public static double goal_angle = -37;

	private final double GRAVITY = DistanceUnit.INCH.fromMeters(9.80665);

	public double desiredVelocity = 0;

	private Pidf pidfController;
	private boolean enabled;
	private double power;

	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;

	private Servo shooterPitchServo;

	private static double pitch = min_pitch;

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
			velocityLUT.new Entry(237.070039065, 1740d),
			velocityLUT.new Entry(225.357833024, 1640d),
			velocityLUT.new Entry(217.076859128, 1520d),
			velocityLUT.new Entry(61.5535989797, 1520d),
			velocityLUT.new Entry(241.221698506, 1800d),
			velocityLUT.new Entry(234.892739735, 1760d),
			velocityLUT.new Entry(264.98179534, 2060d),
			velocityLUT.new Entry(274.09449383, 2160d),
			velocityLUT.new Entry(267.331973714, 2100d),
			velocityLUT.new Entry(282.71322082, 2360d)


			// velocityLUT.new Entry(199.6838915, 1300d),
			// velocityLUT.new Entry(210.7986527, 1460d),
			// velocityLUT.new Entry(217.0768591, 1520d),
			// velocityLUT.new Entry(225.357833, 1620d),
			// velocityLUT.new Entry(237.0700391, 1680d),
			// velocityLUT.new Entry(234.8927397, 1680d),
			// velocityLUT.new Entry(241.2216985, 1790d),
			// velocityLUT.new Entry(252.3611122, 1800d),
			// velocityLUT.new Entry(264.9817953, 1960d),
			// velocityLUT.new Entry(268.4887215, 1980d),
			// velocityLUT.new Entry(275.1588117, 2060d),
			// velocityLUT.new Entry(283.6739618, 2200d)
		);

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
		shooterMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		shooterMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

	public boolean isEnabled() {
		return enabled;
	}

	public void setPower(double pow) {
		pow = Range.clip(pow, -1, 1);
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
		power = pow;
	}

	public static double convertProjectileToShooterVelocity(double projectileVelocity) {
		return velocityLUT.get(projectileVelocity);
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose) {
		updateShooterTarget(robotPose, targetPose, new Vector());
	}

	public void updateShooterTarget(Pose robotPose, Pose targetPose, Vector robotVelocity) {
		double horizontalDistance = robotPose.distanceFrom(targetPose);
		double verticalDistance = goal_height - robot_height;

		double angle = Math.atan((2 * verticalDistance) / horizontalDistance - Math.tan(Math.toRadians(goal_angle)));
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

		desiredVelocity = convertProjectileToShooterVelocity(newV0) + velocityConstant;
		setPitchAngle(Math.toDegrees(newAngle));
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
		return shooterMotorRight.getVelocity();
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
		if (hoodPosition == null || hoodPosition.isNaN()) {
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
