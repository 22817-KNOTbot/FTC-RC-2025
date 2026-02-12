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
	public static double intakeUpPosition = 0;
	public static double intakeDownPosition = 0;

	private DcMotor intakeMotor;
	private Servo intakeLeftServo;
	private Servo intakeRightServo;

	public Intake(HardwareMap hardwareMap) {
		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// intakeLeftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
		// intakeLeftServo.setDirection(Servo.Direction.REVERSE);
		// intakeRightServo = hardwareMap.get(Servo.class, "intakeRightServo");
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
		// intakeLeftServo.setPosition(intakeDownPosition);
		// intakeRightServo.setPosition(intakeDownPosition);
	}

	public void raiseIntake() {
		// intakeLeftServo.setPosition(intakeUpPosition);
		// intakeRightServo.setPosition(intakeUpPosition);
	}
}