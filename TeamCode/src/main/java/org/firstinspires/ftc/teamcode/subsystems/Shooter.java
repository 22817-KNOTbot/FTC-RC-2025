package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Shooter {
	//power = power of shooterMotor
	public static float power = 1;

	private DcMotor shooterMotorLeft;
	private DcMotor shooterMotorRight;

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotor.class, "shooterMotorLeft");
		shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		shooterMotorRight = hardwareMap.get(DcMotor.class, "shooterMotorRight");
		shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	public void enable(boolean enabled) {
		if (enabled) {
			shooterMotorLeft.setPower(power);
			shooterMotorRight.setPower(power);
		} else {
			shooterMotorLeft.setPower(0);
			shooterMotorRight.setPower(0);
		}
	}

	public void setPower(float pow) {
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
	}
}
