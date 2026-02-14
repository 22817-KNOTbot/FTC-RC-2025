package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Intake {
	public static float power = 1f;
	public static double intakeLeftUpPosition = 0.76;
	public static double intakeLeftDownPosition = 0.7;
	public static double intakeRightUpPosition = 0.62;
	public static double intakeRightDownPosition = 0.56;

	private DcMotor intakeMotor;
	private Servo intakeLeftServo;
	private Servo intakeRightServo;

	public Intake(HardwareMap hardwareMap) {
		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		intakeLeftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
		intakeLeftServo.setDirection(Servo.Direction.REVERSE);
		intakeRightServo = hardwareMap.get(Servo.class, "intakeRightServo");
	}

	public void enable(boolean enable) {
		if (enable) {
			intakeMotor.setPower(power);
		} else {
			intakeMotor.setPower(0);
		}
	}

	public void enableReversed(boolean enable) {
		if (enable) {
			intakeMotor.setPower(-power);
		} else {
			intakeMotor.setPower(0);
		}
	}

	public void setPower(float pow) {
		intakeMotor.setPower(pow);
	}

	public void lowerIntake() {
		intakeLeftServo.setPosition(intakeLeftDownPosition);
		intakeRightServo.setPosition(intakeRightDownPosition);
	}

	public void raiseIntake() {
		intakeLeftServo.setPosition(intakeLeftUpPosition);
		intakeRightServo.setPosition(intakeRightUpPosition);
	}
}