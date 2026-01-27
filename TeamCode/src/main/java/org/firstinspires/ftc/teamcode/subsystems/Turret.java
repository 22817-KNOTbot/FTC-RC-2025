package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Turret {
	public static double BASE_ROTATION = 0.4;
	public static double BASE_PITCH = 0.05;

	public static double min_rotation = 0.1;
	public static double max_rotation = 0.7;
	public static double min_pitch = 0.485;
	public static double max_pitch = 0.73;
	public static double rotation_increment = 0.005;
	public static double rotation_per_deg = 0.205/90;
	public static double pitch_increment = 0.01;
	public static double vision_offset = 0.0;

	private AxonServo turretYawServo1;
	private AxonServo turretYawServo2;
	private Servo turretPitchServo;

	private static Position position = Position.BASE;
	private static double rotation = BASE_ROTATION;
	private static double pitch = BASE_PITCH;

	public enum Position {
		BASE,
		MANUAL
	}

	public Turret(HardwareMap hardwareMap) {
		turretYawServo1 = new AxonServo(hardwareMap.get(Servo.class, "turretYawServo1"));
		turretYawServo2 = new AxonServo(hardwareMap.get(Servo.class, "turretYawServo2"));
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

	public double getRotation() {
		Double servo1Angle = turretYawServo1.getAngle();
		Double servo2Angle = turretYawServo2.getAngle() - (turretYawServo2.fullTurnDegrees / 2);

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
			return getTargetRotation();
		}
		return (servo1AngleTurret + servo2AngleTurret) / count;
	}

	public static double getTargetRotation() {
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
		// vector = Range.clip(vector, -1, 1);
		setRotation(rotation + (vector * rotation_increment));
	}

	public void setPitch(double pitch) {
		Turret.position = Position.MANUAL;
		setPitchInternal(pitch);
	}
	
	public void pitchTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setPitch(pitch + (vector * pitch_increment));
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