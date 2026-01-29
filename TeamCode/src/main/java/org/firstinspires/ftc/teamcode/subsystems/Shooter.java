package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Comparator;
import java.util.Iterator;
import java.util.TreeSet;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pidf;

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
	public static double min_angle = 0; // Angle at min_pitch
	// Values are linear units calculated through linkage equation
	public static double arm1_length = 0;
	public static double arm2_length = 0;
	public static double value_per_deg = 0;
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

	public static class VelocityEntries {
		private TreeSet<VelocityEntry> entries;

		public static class VelocityEntry {
			public double distance;
			public double velocity;

			public VelocityEntry(double distance, double velocity) {
				this.distance = distance;
				this.velocity = velocity;
			}
		}

		public VelocityEntries() {
			entries = new TreeSet<VelocityEntry>(new Comparator<VelocityEntry>() {
				@Override
				public int compare(VelocityEntry entry1, VelocityEntry entry2) {
					return (int) Math.signum(entry1.distance - entry2.distance);
				}
			});
		}

		public void add(VelocityEntry entry) {
			entries.add(entry);
		}

		public VelocityEntry[] getNearestEntries(double distance) {
			VelocityEntry compareEntry = new VelocityEntry(distance, 0);
			VelocityEntry lastEntry = entries.floor(compareEntry);
			VelocityEntry nextEntry = entries.ceiling(compareEntry);
			if (lastEntry == null)
				lastEntry = nextEntry;
			if (nextEntry == null)
				nextEntry = lastEntry;
			return new VelocityEntry[] { lastEntry, nextEntry };
		}
	}

	private static final VelocityEntries velocityEntries;

	static {
		Pose goalShooterPose = new Pose(144, 144);
		velocityEntries = new VelocityEntries();
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 72).distanceFrom(goalShooterPose), 1750));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 135).distanceFrom(goalShooterPose), 1600));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 9).distanceFrom(goalShooterPose), 2100));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(48, 9).distanceFrom(goalShooterPose), 2150));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 24).distanceFrom(goalShooterPose), 2000));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 48).distanceFrom(goalShooterPose), 1850));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 96).distanceFrom(goalShooterPose), 1650));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(72, 120).distanceFrom(goalShooterPose), 1600));

		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(96, 96).distanceFrom(goalShooterPose), 1600));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(96, 9).distanceFrom(goalShooterPose), 1980));
		velocityEntries.add(new VelocityEntries.VelocityEntry(new Pose(96, 11).distanceFrom(goalShooterPose), 1900));
	}

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
		shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

	public void updateShooterTarget(Pose robotPose, Pose targetPose, double distance) {
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

	public void updateVelocityTarget(double distance) {
		desiredVelocity = getVelocityTarget(distance);
	}

	public double getVelocityTarget(double distance) {
		// Using linear interpolation
		if (velocityEntries != null) {
			VelocityEntries.VelocityEntry[] nearestEntries = velocityEntries.getNearestEntries(distance);
			VelocityEntries.VelocityEntry lowerEntry = nearestEntries[0];
			VelocityEntries.VelocityEntry higherEntry = nearestEntries[1];

			double distanceDifference = higherEntry.distance - lowerEntry.distance;
			double distanceFraction = (distance - lowerEntry.distance) / distanceDifference;

			double velocityDifference = higherEntry.velocity - lowerEntry.velocity;

			if (distanceDifference != 0) {
				return (distanceFraction * velocityDifference) + lowerEntry.velocity + velocityConstant;
			} else {
				return lowerEntry.velocity + velocityConstant;
			}
		} else {
			return defaultVelocity;
		}
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
		// TODO: Create equation based on testing
		
	}

	public void setPitchLinear(double value) {
		value = Range.clip(value, 0, 1);
		double minPitchValue = (arm1_length * Math.cos(min_pitch)) + Math.sqrt(Math.pow(arm2_length, 2) - (arm1_length * Math.sin(min_pitch)));
		double maxPitchValue = (arm1_length * Math.cos(max_pitch)) + Math.sqrt(Math.pow(arm2_length, 2) - (arm1_length * Math.sin(max_pitch)));
		double scaledValue = Range.scale(value, 0, 1, minPitchValue, maxPitchValue);

		double pitchAngle = Math.acos((Math.pow(arm1_length, 2) + Math.pow(scaledValue, 2) - Math.pow(arm2_length, 2)) / (2 * arm1_length * scaledValue));
		double pitchValue = pitchAngle * value_per_deg - minPitchValue;
		setPitch(pitchValue);
	}

	public void setPitch(double pitchTarget) {
		pitchTarget = Range.clip(pitchTarget, min_pitch, max_pitch);

		shooterPitchServo.setPosition(pitchTarget);

		Shooter.pitch = pitchTarget;
	}
}
