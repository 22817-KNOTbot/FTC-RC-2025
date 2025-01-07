package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Intake {
	/*
	 * Configurable constants
	 *
	 * Most positions/values can be changed here.
	 */
	// Bucket
	public static double BUCKET_INTAKE_HIGH = 0.79;
	public static double BUCKET_INTAKE_LOW = 0.775;

	public static double BUCKET_TRANSFER_POSITION = 0.8;

	public static double BUCKET_POST_TRANSFER_POSITION = 0.83;

	public static double BUCKET_RETRACTED_POSITION = 0.87;

	// Slides
	public static int SLIDE_POSITION_MIN = 600;
	public static int SLIDE_POSITION_DEFAULT = 800;
	public static int SLIDE_POSITION_MAX = 800;

	public static int SLIDE_TRANSFER_POSITION = 0;
	
	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private Servo flipServoLeft;
	private Servo flipServoRight;
	private DcMotor intakeSlides;
	private DcMotor intakeMotor;
	private TouchSensor intakeTouch;
	private ColorRangeSensor colourRangeSensor;

	public static Positions bucketPosition = Positions.INTAKE;
	public static Positions slidePosition = Positions.TRANSFER;

	public enum Positions {
		RETRACTED,
		INTAKE,
		TRANSFER,
		POST_TRANSFER,
		MANUAL
	}

	public enum SampleColours {
		RED,
		BLUE,
		YELLOW
	}

	public Intake(HardwareMap hardwareMap, boolean resetEncoder) {
		flipServoLeft = hardwareMap.get(Servo.class, "flipServoLeft");
		flipServoRight = hardwareMap.get(Servo.class, "flipServoRight");
		flipServoLeft.setDirection(Servo.Direction.FORWARD);
		flipServoRight.setDirection(Servo.Direction.REVERSE);

		// 0.012287150712598427 inPerTick
		intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		if (resetEncoder) intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeSlides.setTargetPosition(0);
		intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
	}

	public void abort() {
		intakeMotor.setPower(0);
		intakeSlides.setPower(0);
	}

	/*
	 * Sets the position of intake
	 * 
	 * There are 5 methods: 
	 * 	 master, (changes everything) 
	 * 	 enum bucket, 
	 * 	 manual bucket, 
	 * 	 enum slides, 
	 * 	 manual slides
	 * Also some misc. get functions
	 * 
	 * To change positions, set the configurable constants at the top
	 */
	public void setPosition(Positions position) {
		setBucketPosition(position);
		setSlidePosition(position);
	}

	public void setBucketPosition(Positions position) {
		Intake.bucketPosition = position;
		double target = 0;
		switch (position) {
			case RETRACTED:
				target = BUCKET_RETRACTED_POSITION;
				break;
			case INTAKE:
				target = BUCKET_INTAKE_LOW;
				break;
			case TRANSFER:
			default:
				target = BUCKET_TRANSFER_POSITION;
				break;
			case POST_TRANSFER:
				target = BUCKET_POST_TRANSFER_POSITION;
				break;
		}

		flipServoLeft.setPosition(target);
		flipServoRight.setPosition(target);
	}
	
	public void setBucketPosition(double position) {
		Intake.bucketPosition = Intake.Positions.MANUAL;
		flipServoLeft.setPosition(position);
		flipServoRight.setPosition(position);
	}

	public void setSlidePosition(Positions position) {
		Intake.slidePosition = position;
		int target = 0;
		switch (position) {
			case INTAKE:
				target = SLIDE_POSITION_DEFAULT;
				break;
			case TRANSFER:
			case POST_TRANSFER:
			default:
				target = SLIDE_TRANSFER_POSITION;
				break;
		}
		intakeSlides.setPower(1);
		intakeSlides.setTargetPosition(target);
	}
	
	public void setSlidePosition(int position) {
		Intake.slidePosition = Intake.Positions.MANUAL;
		intakeSlides.setPower(1);
		intakeSlides.setTargetPosition(position);
	}

	public int getSlidePosition() {
		return intakeSlides.getCurrentPosition();
	}

	public boolean isSlideBusy() {
		return intakeSlides.isBusy();
	}

	public void setSlidePower(double power) {
		intakeSlides.setPower(power);
	}

	/*
	 * Sets power of intake motor
	 */
	public void setPower(double power) {
		intakeMotor.setPower(power);
	}

	/*
	 * Methods for sensor input: colour/range sensor and touch sensor
	 * 
	 * Should never need to be changed, except possibly thresholds of
	 * the below method
	 */
	public boolean checkSample(SampleColours colour) {
		int red = getRed();
		int green = getGreen();
		int blue = getBlue();
		boolean correct = false;
		if (colourSensorResponding()) {
			switch (colour) {
				case RED:
					correct = (red / green > 1.6) && (red / blue > 2);
					break;
				case BLUE:
					correct = (blue / red > 3.5) && (blue / green > 1.2);
					break;
				case YELLOW:
					correct = (red / blue > 2.5) && (green / blue > 3);
					break;
			}
		} else {
			correct = true;
		}
		return correct;
	}

	public int getRed() {
		return colourRangeSensor.red();
	}

	public int getGreen() {
		return colourRangeSensor.green();
	}

	public int getBlue() {
		return colourRangeSensor.blue();
	}

	public double getDistance(DistanceUnit unit) {
		return colourRangeSensor.getDistance(unit);
	}

	public boolean colourSensorResponding() {
		// TODO: Find better way to detect disconnect
		return !(colourRangeSensor == null || (getRed() == 0 && getGreen() == 0 && getBlue() == 0));
	}

	public boolean isTouched() {
		return intakeTouch.isPressed();
	}
}