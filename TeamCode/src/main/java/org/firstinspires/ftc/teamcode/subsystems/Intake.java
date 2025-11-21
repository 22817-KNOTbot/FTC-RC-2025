package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Intake {
	public static float power = 0.8f;

	private DcMotor intakeMotor;

	public Intake(HardwareMap hardwareMap) {
		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setDirection(DcMotor.Direction.REVERSE);
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
}