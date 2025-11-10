package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
public class Shooter {
	//power = power of shooterMotor
	public static float power = 1;
	public static double shooterVelocity = 5;

	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;

	public Shooter(HardwareMap hardwareMap) {
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
		//shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		shooterMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
		shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
	}
	public void enable(boolean enabled) {
		if (enabled) {
			shooterMotorLeft.setVelocity(shooterVelocity);
			shooterMotorRight.setPower(shooterMotorLeft.getPower());
		} else {
			shooterMotorLeft.setPower(0);
			shooterMotorRight.setPower(0);
		}
	}

	public void setPower(float pow) {
		shooterMotorLeft.setPower(pow);
		shooterMotorRight.setPower(pow);
	}

	public double getVelocityLeft() {
		return shooterMotorLeft.getVelocity();
	}
	public double getVelocityRight() {
		return shooterMotorRight.getVelocity();
	}

	public double getVelocity() {
		return shooterMotorLeft.getVelocity();
	}
}
