package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.VelocityEntries.VelocityEntry;
import org.firstinspires.ftc.teamcode.util.Alliance;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Shooter {
	// power = power of shooterMotor
	public static float power = 1;
	// See Desmos graph for regression. Constants for cubic regression
	// public static double velocityEquationCoefficient_3 = 0.00169552;
	// public static double velocityEquationCoefficient_2 = -0.36759;
	// public static double velocityEquationCoefficient_1 = 29.06798;
	// public static double velocityEquationConstant = 838.74397;
	public static double velocityConstant = 50;
	public static double velocityTargetOffset = 50;
	public static double shootingAreaTolerance = 12.7279220614;
	public static double defaultVelocity = 2200;
	public static VelocityEntries velocityEntries;

	public double desiredVelocity = 2200;
	public double targetVelocity = desiredVelocity + velocityTargetOffset;

	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;

	public static class VelocityEntries {
		private SortedSet<VelocityEntry> entries;

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
			Iterator<VelocityEntry> iterator = entries.iterator();
			VelocityEntry lastEntry = null;
			VelocityEntry nextEntry = null;
			while (iterator.hasNext()) {
				VelocityEntry currentEntry = iterator.next();
				if (distance < currentEntry.distance) {
					nextEntry = currentEntry;
					break;
				} else {
					lastEntry = currentEntry;
				}
			}
			if (lastEntry == null)
				lastEntry = nextEntry;
			if (nextEntry == null)
				nextEntry = lastEntry;
			return new VelocityEntry[] { lastEntry, nextEntry };
		}
	}

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
		// shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
		shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

		addVelocityEntries();
	}

	public void addVelocityEntries() {
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
	}

	public void enable(boolean enabled) {
		if (enabled) {
			shooterMotorLeft.setVelocity(targetVelocity);
			shooterMotorRight.setPower(shooterMotorLeft.getPower());
		} else {
			shooterMotorLeft.setPower(0);
			shooterMotorRight.setPower(0);
		}
	}

	public void setPower(float pow) {
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
	}

	public void updateVelocity(double distance) {
		// Using linear interpolation
		if (velocityEntries != null) {
			VelocityEntries.VelocityEntry[] nearestEntries = velocityEntries.getNearestEntries(distance);
			VelocityEntries.VelocityEntry lowerEntry = nearestEntries[0];
			VelocityEntries.VelocityEntry higherEntry = nearestEntries[1];

			double distanceDifference = higherEntry.distance - lowerEntry.distance;
			double distanceFraction = (distance - lowerEntry.distance) / distanceDifference;

			double velocityDifference = higherEntry.velocity - lowerEntry.velocity;

			if (distanceDifference != 0) {
				desiredVelocity = (distanceFraction * (velocityDifference)) + lowerEntry.velocity + velocityConstant;
			} else {
				desiredVelocity = lowerEntry.velocity + velocityConstant;
			}
		} else {
			desiredVelocity = defaultVelocity;
		}
		targetVelocity = desiredVelocity + velocityTargetOffset;

		if (shooterMotorLeft.getPower() > 0) {
			enable(true);
		}
	}

	public boolean atDesiredVelocity() {
		return getVelocity() >= desiredVelocity;
	}

	public double getVelocity() {
		return shooterMotorLeft.getVelocity();
	}
}
