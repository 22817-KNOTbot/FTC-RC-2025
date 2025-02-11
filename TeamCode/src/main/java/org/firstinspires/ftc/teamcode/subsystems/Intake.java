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
	// Intake
	public static double RETRACTED_DRIVE = 0.57;
	public static double RETRACTED_WRIST = 0.56;

	public static double PRE_INTAKE_DRIVE = 0.502;
	public static double PRE_INTAKE_WRIST = 0.46;

	public static double INTAKE_DRIVE = 0.49;
	public static double INTAKE_WRIST = 0.46;

	public static double POST_INTAKE_DRIVE = 0.52;
	public static double POST_INTAKE_WRIST = 0.48;

	public static double TRANSFER_DRIVE = 0.527;
	public static double TRANSFER_WRIST = 0.553;

	public static double WRIST_VALUE_PER_DEG = 0.0007555556;

	// Slides
	public static int INTAKE_SLIDE_POSITION = 350;

	public static int SLIDE_TRANSFER_POSITION = 0;

	// Claw
	public static double CLAW_OPEN = 0.395;
	public static double CLAW_CLOSED = 0.05;
	
	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private Servo intakeDriveServoLeft;
	private Servo intakeDriveServoRight;
	private Servo intakeWristServoLeft;
	private Servo intakeWristServoRight;
	private Servo intakeClaw;
	private DcMotor intakeSlides;
	private ColorRangeSensor colourRangeSensor;
	public double wristPosition = 0;

	public static Positions intakePosition = Positions.INTAKE;
	public static Positions slidePosition = Positions.TRANSFER;

	public enum Positions {
		RETRACTED,
		PRE_INTAKE,
		INTAKE,
		POST_INTAKE,
		TRANSFER,
		MANUAL
	}

	public enum SampleColours {
		RED,
		BLUE,
		YELLOW
	}

	public Intake(HardwareMap hardwareMap, boolean resetEncoder) {
		intakeDriveServoLeft = hardwareMap.get(Servo.class, "intakeDriveServoLeft");
		intakeDriveServoRight = hardwareMap.get(Servo.class, "intakeDriveServoRight");
		intakeDriveServoLeft.setDirection(Servo.Direction.REVERSE);
		intakeDriveServoRight.setDirection(Servo.Direction.FORWARD);

		intakeWristServoLeft = hardwareMap.get(Servo.class, "intakeWristServoLeft");
		intakeWristServoRight = hardwareMap.get(Servo.class, "intakeWristServoRight");
		intakeWristServoLeft.setDirection(Servo.Direction.REVERSE);
		intakeWristServoRight.setDirection(Servo.Direction.FORWARD);

		intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

		// 0.012287150712598427 inPerTick
		intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		if (resetEncoder) intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeSlides.setTargetPosition(0);
		intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
	}

	public void abort() {
		intakeSlides.setPower(0);
	}

	/*
	 * Sets the position of intake
	 * 
	 * There are 5 methods: 
	 * 	 master, (changes everything) 
	 * 	 enum intake, 
	 * 	 manual intake, 
	 * 	 enum slides, 
	 * 	 manual slides
	 * Also some misc. get functions
	 * 
	 * To change positions, set the configurable constants at the top
	 */
	public void setPosition(Positions position) {
		setIntakePosition(position);
		setSlidePosition(position);
	}

	public void setIntakePosition(Positions position) {
		Intake.intakePosition = position;
		double driveTarget = 0;
		double wristTarget = 0;
		switch (position) {
			case RETRACTED:
				driveTarget = RETRACTED_DRIVE;
				wristTarget = RETRACTED_WRIST;
				break;
			case PRE_INTAKE:
				driveTarget = PRE_INTAKE_DRIVE;
				wristTarget = PRE_INTAKE_WRIST;
				break;
			case INTAKE:
				driveTarget = INTAKE_DRIVE;
				wristTarget = INTAKE_WRIST;
				break;
			case POST_INTAKE:
				driveTarget = POST_INTAKE_DRIVE;
				wristTarget = POST_INTAKE_WRIST;
				break;
			case TRANSFER:
			default:
				driveTarget = TRANSFER_DRIVE;
				wristTarget = TRANSFER_WRIST;
				break;
		}

		intakeDriveServoLeft.setPosition(driveTarget);
		intakeDriveServoRight.setPosition(driveTarget);

		if (wristPosition != wristTarget) {
			intakeWristServoLeft.setPosition(wristTarget);
			intakeWristServoRight.setPosition(wristTarget);
			wristPosition = wristTarget;
		}
	}
	
	public void setIntakePosition(double driveTarget, double wristTarget) {
		Intake.intakePosition = Intake.Positions.MANUAL;
		intakeDriveServoLeft.setPosition(driveTarget);
		intakeDriveServoRight.setPosition(driveTarget);

		if (wristPosition != wristTarget) {
			intakeWristServoLeft.setPosition(wristTarget);
			intakeWristServoRight.setPosition(wristTarget);
			wristPosition = wristTarget;
		}
	}

	public void setWristRotation(double rotation) {
		if (wristPosition + rotation > 1 || wristPosition - rotation < 0) return;
		intakeWristServoLeft.setPosition(wristPosition + rotation);
		intakeWristServoRight.setPosition(wristPosition - rotation);
	}

	public void setSlidePosition(Positions position) {
		Intake.slidePosition = position;
		int target = 0;
		switch (position) {
			case PRE_INTAKE:
			case INTAKE:
				target = INTAKE_SLIDE_POSITION;
				break;
			case POST_INTAKE:
			case TRANSFER:
			case RETRACTED:
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
	
	// Alternate method which should be faster than the native isBusy()
	// Compares current position to target position
	public boolean isSlideBusyFast() {
		return isSlideBusyFast(10);
	}

	public boolean isSlideBusyFast(int dist) {
		return Math.abs(intakeSlides.getCurrentPosition() - intakeSlides.getTargetPosition()) > dist;
	}

	public void setSlidePower(double power) {
		intakeSlides.setPower(power);
	}

	/*
	 * Open/closes claw
	 */
	public void openClaw() {
		intakeClaw.setPosition(CLAW_OPEN);
	}

	public void closeClaw() {
		intakeClaw.setPosition(CLAW_CLOSED);
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
					correct = (red > green) && (green > blue);
					break;
				case BLUE:
					correct = (blue > green) && (green > red);
					break;
				case YELLOW:
					correct = (green > 100) && (green > red) && (red > blue);
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
}