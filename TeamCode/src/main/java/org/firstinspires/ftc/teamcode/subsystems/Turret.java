package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Turret {
	public static double BASE_ROTATION = 0.5;

	public static double min_rotation = 0.0;
	public static double max_rotation = 1.0;
	public static double rotation_increment = 0.02;
	public static double rotation_per_deg = 0.25/90;
	public static double vision_offset = 0;
	public static double servo_threshold = 0.001;

	public static double Kp = 0.00023;
	public static double Ki = 0.0000013;
	public static double Kd = 0.000012;

	private AxonServo turretYawServo1;
	private AxonServo turretYawServo2;

	private static double rotation = BASE_ROTATION;

	public Turret(HardwareMap hardwareMap) {
		turretYawServo1 = new AxonServo(hardwareMap.get(Servo.class, "turretYawServo1"));
		turretYawServo2 = new AxonServo(hardwareMap.get(Servo.class, "turretYawServo2"));
		turretYawServo1.setDirection(Servo.Direction.FORWARD);
		turretYawServo2.setDirection(Servo.Direction.FORWARD);
	}

	public void abort() {
		// Currently does nothing
		// Exists for future use
	}

	public void update() {
		turretYawServo1.update();
		turretYawServo2.update();
	}

	/*
	 * Getter methods
	 */

	public double getRotation() {
		Double servo1Angle = turretYawServo1.getAngle();
		Double servo2Angle = turretYawServo2.getAngle();

		double count = 0;
		double servo1AngleTurret = 0;
		double servo2AngleTurret = 0;

		if (servo1Angle != null) {
			count++;
			servo1AngleTurret = servo1Angle - (turretYawServo1.fullTurnDegrees / 2);
		}
		if (servo2Angle != null) {
			count++;
			servo2AngleTurret = servo2Angle - (turretYawServo2.fullTurnDegrees / 2);
		}
		if (count == 0) {
			return (getTargetRotation() - Turret.BASE_ROTATION) * rotation_per_deg;
		}
		return (servo1AngleTurret + servo2AngleTurret) / count;
	}

	public static double getTargetRotationAngle() {
		return Turret.getTargetRotation() / Turret.rotation_per_deg - 180;
	}

	public static double getTargetRotation() {
		return Turret.rotation;
	}

	/*
	 * Turret
	 */

	public void setRotation(double rotationTarget) {
		setRotationInternal(rotationTarget);
	}

	public void rotateTurret(double vector) {
		// vector = Range.clip(vector, -1, 1);
		setRotation(rotation + (vector * rotation_increment));
	}

	/*
	 * Internal functions
	 */

	private void setRotationInternal(double rotationTarget) {
		rotationTarget = Range.clip(rotationTarget, min_rotation, max_rotation);
		if (Math.abs(rotationTarget - rotation) <= servo_threshold) {
			return;
		}
		turretYawServo1.setPosition(rotationTarget);
		turretYawServo2.setPosition(rotationTarget);

		Turret.rotation = rotationTarget;
	}
}