package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Turret {
	public static double BASE_HOOD = 0;
	public static int BASE_ROTATION = 0;

	public static int min_rotation = 0;
	public static int max_rotation = 0;
	public static double min_hood_pitch = 0.5;
	public static double max_hood_pitch = 0.5;
	public static int rotation_increment = 100;

	private DcMotor turretDiffMotor1;
	private DcMotor turretDiffMotor2;
	private Servo turretHoodServo;

	public static Position position = Position.BASE;
	public static int rotation = 0;

	public enum Position {
		BASE,
		MANUAL
	}

	public Turret(HardwareMap hardwareMap, boolean resetEncoders) {
		turretDiffMotor1 = hardwareMap.get(DcMotor.class, "turretDiffMotor1");
		turretDiffMotor2 = hardwareMap.get(DcMotor.class, "turretDiffMotor2");
		if (resetEncoders) {
			turretDiffMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			turretDiffMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		turretDiffMotor1.setDirection(DcMotor.Direction.FORWARD);
		turretDiffMotor2.setDirection(DcMotor.Direction.FORWARD);
		turretDiffMotor1.setTargetPosition(0);
		turretDiffMotor2.setTargetPosition(0);
		turretDiffMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turretDiffMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turretDiffMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turretDiffMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		turretHoodServo = hardwareMap.get(Servo.class, "turretHoodServo");
		turretHoodServo.setDirection(Servo.Direction.FORWARD);
	}

	public void abort() {
		turretDiffMotor1.setPower(0);
		turretDiffMotor2.setPower(0);
	}

	/*
	 * Turret
	 */

	public void setPosition(Position position) {
		Turret.position = position;
		int rotationTarget = 0;
		double hoodTarget = 0;

		switch (position) {
			case BASE:
				rotationTarget = BASE_ROTATION;
				hoodTarget = BASE_HOOD;
				break;
			case MANUAL:
				break;
		}

		setPositionInternal(rotationTarget, hoodTarget);
	}

	public void setRotation(int rotationTarget) {
		Turret.position = Position.MANUAL;
		setRotationInternal(rotationTarget);
	}

	public void rotateTurret(double vector) {
		vector = Range.clip(vector, -1, 1);
		setRotation(rotation + (int) Math.round(vector * rotation_increment));
	}

	public void setPitch(double pitch) {
		Turret.position = Position.MANUAL;
		pitch = Range.clip(pitch, 0, 1);
		setPitchInternal(pitch);
	}

	/*
	 * Internal functions
	 */

	private void setPositionInternal(int rotationTarget, double hoodTarget) {
		setRotationInternal(rotationTarget);
		setPitchInternal(hoodTarget);
	}

	private void setRotationInternal(int rotationTarget) {
		rotationTarget = Range.clip(rotationTarget, min_rotation, max_rotation);

		turretDiffMotor1.setTargetPosition(rotationTarget);
		turretDiffMotor2.setTargetPosition(rotationTarget);

		rotation += rotationTarget;
	}

	private void setPitchInternal(double hoodTarget) {
		hoodTarget = Range.clip(hoodTarget, min_hood_pitch, max_hood_pitch);

		turretHoodServo.setPosition(hoodTarget);
	}
}