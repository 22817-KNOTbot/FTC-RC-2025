package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Shooter {
	//power = power of shooterMotor
	public static float power = 1;
	// See Desmos graph for regression. Constants for cubic regression
	public static double velocityEquationCoefficient_3 = 0.000637352;
	public static double velocityEquationCoefficient_2 = -0.199581;
	public static double velocityEquationCoefficient_1 = 31.93207;
	public static double velocityEquationConstant = -100;
	public static int velocityTargetOffset = 50;
	public static double shootingAreaTolerance = 12.7279220614;
	
	public double desiredVelocity = 2200;
	public double targetVelocity = desiredVelocity + velocityTargetOffset;

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
			shooterMotorLeft.setVelocity(targetVelocity);
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

	public void updateVelocity(double distance) {
		// Using cubic regression
		desiredVelocity = velocityEquationCoefficient_3 * Math.pow(distance, 3)
				+ velocityEquationCoefficient_2 * Math.pow(distance, 2)
				+ velocityEquationCoefficient_1 * distance
				+ velocityEquationConstant;
		targetVelocity = desiredVelocity + velocityTargetOffset;

		if (shooterMotorLeft.getPower() > 0) {
			enable(true);
		}
	}

	public boolean atDesiredVelocity() {
		return getVelocity() >= desiredVelocity;
	}

	public double getVelocity() {
		return shooterMotorLeft.getVelocity();
	}
}
