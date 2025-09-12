package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Shooter {
	//power = power of shooterMotor
	public static float power = 1;

	private DcMotor shooterMotor;

	public Shooter(HardwareMap hardwareMap) {
		shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
		shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	public void enable(boolean enabled) {
		if (enabled) {
			shooterMotor.setPower(power);
		} else {
			shooterMotor.setPower(0);
		}
	}

	public void setPower(float pow) {
		shooterMotor.setPower(pow);
	}
}
