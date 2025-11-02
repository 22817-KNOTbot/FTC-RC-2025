package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Turret {
	public static double BASE_ROTATION = 0.5;
	public static double BASE_PITCH = 0.05;

	public static double min_rotation = 0.3;
	public static double max_rotation = 0.7;
	public static double min_pitch = 0.05;
	public static double max_pitch = 0.6;
	public static double rotation_increment = 0.0005;
	public static double rotation_per_deg = (1d/1800)*3;
	public static double pitch_incremenet = 0.01;

	private Servo turretYawServo1;
	private Servo turretYawServo2;
	private Servo turretPitchServo;

	private static Position position = Position.BASE;
	private static double rotation = BASE_ROTATION;
	private static double pitch = BASE_PITCH;

	public enum Position {
		BASE,
		MANUAL
	}

	public Turret(HardwareMap hardwareMap) {
		turretYawServo1 = hardwareMap.get(Servo.class, "turretYawServo1");
		turretYawServo2 = hardwareMap.get(Servo.class, "turretYawServo2");
		turretYawServo1.setDirection(Servo.Direction.FORWARD);
		turretYawServo2.setDirection(Servo.Direction.FORWARD);
		turretPitchServo = hardwareMap.get(Servo.class, "turretPitchServo");
		turretPitchServo.setDirection(Servo.Direction.FORWARD);
	}

	public void abort() {
		// Currently does nothing
		// Exists for future use
	}

	/*
	 * Getter methods
	 */

	public static Position getPosition() {
		return Turret.position;
	}

	public static double getRotation() {
		return Turret.rotation;
	}

	public static double getPitch() {
		return Turret.pitch;
	}

	/*
	 * Turret
	 */

	public void setPosition(Position position) {
		Turret.position = position;
		double rotationTarget = 0;
		double pitchTarget = 0;

		switch (position) {
			case BASE:
				rotationTarget = BASE_ROTATION;
				pitchTarget = BASE_PITCH;
				break;
			case MANUAL:
				break;
		}

		setPositionInternal(rotationTarget, pitchTarget);
	}

	public void setRotation(double rotationTarget) {
		Turret.position = Position.MANUAL;
		setRotationInternal(rotationTarget);
	}

	public void rotateTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setRotation(rotation + (vector * rotation_increment));
	}

	public void setPitch(double pitch) {
		Turret.position = Position.MANUAL;
		setPitchInternal(pitch);
	}
	
	public void pitchTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setPitch(pitch + (vector * pitch_incremenet));
	}

	/*
	 * Internal functions
	 */

	private void setPositionInternal(double rotationTarget, double pitchTarget) {
		setRotationInternal(rotationTarget);
		setPitchInternal(pitchTarget);
	}

	private void setRotationInternal(double rotationTarget) {
		rotationTarget = Range.clip(rotationTarget, min_rotation, max_rotation);

		turretYawServo1.setPosition(rotationTarget);
		turretYawServo2.setPosition(rotationTarget);

		Turret.rotation = rotationTarget;
	}

	private void setPitchInternal(double pitchTarget) {
		pitchTarget = Range.clip(pitchTarget, min_pitch, max_pitch);

		turretPitchServo.setPosition(pitchTarget);

		Turret.pitch = pitchTarget;
	}
}