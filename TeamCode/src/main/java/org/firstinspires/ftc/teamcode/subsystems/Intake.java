package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Intake {
	public static double power = 1;
	public static double power_slow = 0.65;
	public static double distance_threshold_mm = 20;
	public static double sensor_cache_time_ms = 20;
	public static double loaded_full_ms = 1500;

	private DcMotor intakeMotor;
	private ColorRangeSensor intakeSensor;

	private boolean enabled;
	private double lastSensorDistance;
	private double lastSensorTime;
	private ElapsedTime intakeLoadedTimer;

	public Intake(HardwareMap hardwareMap) {
		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensor");
	}

	public void enable(boolean enable) {
		if (enable) {
			intakeMotor.setPower(power);
		} else {
			intakeMotor.setPower(0);
		}
		this.enabled = enable;
	}

	public void enableSlow(boolean enable) {
		if (enable) {
			intakeMotor.setPower(power_slow);
		} else {
			intakeMotor.setPower(0);
		}
		this.enabled = enable;
	}

	public void enableReversed(boolean enable) {
		if (enable) {
			intakeMotor.setPower(-power);
		} else {
			intakeMotor.setPower(0);
		}
		this.enabled = enable;
	}

	public void setPower(float pow) {
		intakeMotor.setPower(pow);
		enabled = pow != 0;
	}

	public boolean isEnabled() {
		return enabled;
	}

	/**
	 * Updates the intake sensor to check if intake is loaded.
	 * Should be called repeatedly while intaking.
	 * Only returns true once when the intake is loaded for at least {@link #loaded_full_ms} milliseconds.
	 * @return Whether the intake has been loaded for at least {@link #loaded_full_ms} milliseconds.
	 */
	public boolean intakeUpdate() {
		updateCachedDistance();
		if (lastSensorDistance <= distance_threshold_mm) {
			if (intakeLoadedTimer == null) {
				intakeLoadedTimer = new ElapsedTime();
			} else if (intakeLoadedTimer.milliseconds() >= loaded_full_ms) {
				intakeLoadedTimer = null;
				return true;
			}
		}
		return false;
	}

	public void intakeReset() {
		intakeLoadedTimer = null;
	}

	public boolean getLoaded() {
		return updateCachedDistance() <= distance_threshold_mm;
	}

	public double updateCachedDistance() {
		if (System.currentTimeMillis() - lastSensorTime >= sensor_cache_time_ms) {
			lastSensorDistance = intakeSensor.getDistance(DistanceUnit.MM);
			lastSensorTime = System.currentTimeMillis();
		}
		return lastSensorDistance;
	}
}