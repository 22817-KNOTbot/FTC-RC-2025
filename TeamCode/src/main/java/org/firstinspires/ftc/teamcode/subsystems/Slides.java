package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Slides {
	/*
	 * Configurable constants
	 *
	 * Most positions/values can be changed here.
	 */
	public static int RETRACTED_POSITION = 0;

	public static int LOW_BASKET_POSITION = 800;
	public static int HIGH_BASKET_POSITION = 2250;

	public static int HIGH_CHAMBER_PREHANG_POSITION = 400;
	public static int HIGH_CHAMBER_HANG_POSITION = 850;

	public static int ASCEND_ONE_POSITION = 2000;

	public static int ASCEND_TWO_PRE_POSITION = 2200;
	public static int ASCEND_TWO_RETRACT_POSITION = 1500;

	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private DcMotor slideMotorLeft;
	private DcMotor slideMotorRight;

	public static Positions position = Positions.RETRACTED;

	public enum Positions {
		RETRACTED,
		LOW_BASKET,
		HIGH_BASKET,
		HIGH_CHAMBER_PREHANG,
		HIGH_CHAMBER_HANG,
		ASCEND_ONE,
		ASCEND_TWO_PRE,
		ASCEND_TWO_RETRACT,
		MANUAL
	}

	public Slides(HardwareMap hardwareMap, boolean resetEncoder) {
		slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotorLeft.setTargetPosition(0);
		slideMotorRight.setTargetPosition(0);
		slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		if (resetEncoder) {
			slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/*
	 * Sets the position of the slides
	 * 
	 * There are 2 methods: enum and manual.
	 * Enum should be the primary one used, 
	 * as it also updates a variable for telemetry
	 * and allows for quick changing of positions.
	 */

	public void setPosition(Positions position) {
		Slides.position = position;
		int target = 0;
		switch (position) {
			case RETRACTED:
			default:
				target = RETRACTED_POSITION;
				break;
			case LOW_BASKET:
				target = LOW_BASKET_POSITION;
				break;
			case HIGH_BASKET:
				target = HIGH_BASKET_POSITION;
				break;
			case HIGH_CHAMBER_PREHANG:
				target = HIGH_CHAMBER_PREHANG_POSITION;
				break;
			case HIGH_CHAMBER_HANG:
				target = HIGH_CHAMBER_HANG_POSITION;
				break;
			case ASCEND_ONE:
				target = ASCEND_ONE_POSITION;
				break;
			case ASCEND_TWO_PRE:
				target = ASCEND_TWO_PRE_POSITION;
				break;
			case ASCEND_TWO_RETRACT:
				target = ASCEND_TWO_RETRACT_POSITION;
				break;
		}
		slideMotorLeft.setPower(1);
		slideMotorRight.setPower(1);
		slideMotorLeft.setTargetPosition(target);
		slideMotorRight.setTargetPosition(target);
	}

	public void setPosition(int position) {
		Slides.position = Positions.MANUAL;
		slideMotorLeft.setPower(1);
		slideMotorRight.setPower(1);
		slideMotorLeft.setTargetPosition(position);
		slideMotorRight.setTargetPosition(position);
	}

	public void setPower(double power) {
		slideMotorLeft.setPower(power);
		slideMotorRight.setPower(power);
	}

	public int getSlideLeftPosition() {
		return slideMotorLeft.getCurrentPosition();
	}

	public int getSlideRightPosition() {
		return slideMotorRight.getCurrentPosition();
	}

	public boolean isSlideBusy() {
		return slideMotorLeft.isBusy() || slideMotorRight.isBusy();
	}

		// Alternate method which should be faster than the native isBusy()
	// Compares current position to target position
	public boolean isSlideBusyFast() {
		return isSlideBusyFast(10);
	}

	public boolean isSlideBusyFast(int dist) {
		boolean leftBusy = Math.abs(slideMotorLeft.getCurrentPosition() - slideMotorLeft.getTargetPosition()) > dist;
		boolean rightBusy = Math.abs(slideMotorRight.getCurrentPosition() - slideMotorRight.getTargetPosition()) > dist;
		return leftBusy || rightBusy;
	}
}
