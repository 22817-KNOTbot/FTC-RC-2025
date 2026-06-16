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
	public static double distance_left_threshold_mm = 28;
	public static double distance_right_threshold_mm = 35;
	public static double sensor_cache_time_ms = 20;
	public static double loaded_full_ms = 0;

	private DcMotor intakeMotor;
	private ColorRangeSensor intakeLeftSensor;
	private ColorRangeSensor intakeRightSensor;

	private boolean enabled;
	private double lastLeftSensorDistance;
	private double lastRightSensorDistance;
	private double lastSensorTime;
	private ElapsedTime intakeLoadedTimer;

	public Intake(HardwareMap hardwareMap) {
		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		intakeLeftSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensorLeft");
		intakeRightSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensorRight");
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
		if (getLoaded()) {
			if (intakeLoadedTimer == null) {
				intakeLoadedTimer = new ElapsedTime();
			} else if (intakeLoadedTimer.milliseconds() >= loaded_full_ms) {
				intakeLoadedTimer = null;
				return true;
			}
		} else {
			intakeLoadedTimer = null;
		}
		return false;
	}

	public void intakeReset() {
		intakeLoadedTimer = null;
	}

	public boolean getLoaded() {
		updateCachedDistance();
		return lastLeftSensorDistance <= distance_left_threshold_mm
				&& lastRightSensorDistance <= distance_right_threshold_mm;
	}

	public boolean getAnyLoaded() {
		updateCachedDistance();
		return lastLeftSensorDistance <= distance_left_threshold_mm
				|| lastRightSensorDistance <= distance_right_threshold_mm;
	}

	public void updateCachedDistance() {
		if (System.currentTimeMillis() - lastSensorTime >= sensor_cache_time_ms) {
			lastLeftSensorDistance = intakeLeftSensor.getDistance(DistanceUnit.MM);
			lastRightSensorDistance = intakeRightSensor.getDistance(DistanceUnit.MM);
			lastSensorTime = System.currentTimeMillis();
		}
	}
}