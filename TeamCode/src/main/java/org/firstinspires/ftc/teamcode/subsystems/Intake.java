package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
	public static boolean on = false;
	public static float power = 1;

	private DcMotor intake;

	public Intake(HardwareMap hardwareMap) {
		intake = hardwareMap.get(DcMotor.class, "intake");
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	public void enable(boolean enable) {
		if (!on && enable) {
			intake.setPower(power);
		} else if (on && !enable) {
			intake.setPower(0);
		}
	}

	public void setPower(float pow) {
		intake.setPower(pow);
	}
}