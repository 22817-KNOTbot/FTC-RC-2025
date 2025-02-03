package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
	public static double RETRACTED_DRIVE = 0.985;
	public static double RETRACTED_COAX = 0;

	public static double PRE_INTAKE_DRIVE = 0.915;
	public static double PRE_INTAKE_COAX = 0;

	public static double INTAKE_DRIVE = 0.9025;
	public static double INTAKE_COAX = 0;

	public static double TRANSFER_DRIVE = 0.925;
	public static double TRANSFER_COAX = 0;

	public static double offset_coax = 0;

	// Slides
	public static int SLIDE_POSITION_MIN = 0;
	public static int SLIDE_POSITION_DEFAULT = 800;
	public static int SLIDE_POSITION_MAX = 1200;

	public static int SLIDE_TRANSFER_POSITION = 0;

	// Wrist
	public static double WRIST_MIDDLE_POSITION = 0.5;

	// Claw
	public static double CLAW_OPEN = 0.6;
	public static double CLAW_CLOSED = 0.4;
	
	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private Servo intakeDriveServoLeft;
	private Servo intakeDriveServoRight;
	private Servo intakeCoaxServo;
	private Servo intakeClaw;
	private ServoImplEx intakeWrist;
	private DcMotor intakeSlides;
	private ColorRangeSensor colourRangeSensor;

	public static Positions intakePosition = Positions.INTAKE;
	public static Positions slidePosition = Positions.TRANSFER;

	public enum Positions {
		RETRACTED,
		PRE_INTAKE,
		INTAKE,
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
		intakeDriveServoLeft.setDirection(Servo.Direction.FORWARD);
		intakeDriveServoRight.setDirection(Servo.Direction.REVERSE);

		intakeCoaxServo = hardwareMap.get(Servo.class, "intakeCoaxServo");

		intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
		
		intakeWrist = hardwareMap.get(ServoImplEx.class, "intakeWrist");
		intakeWrist.setPwmRange(new ServoImplEx.PwmRange(500, 2500));
		intakeWrist.scaleRange(0.5, 0.7);

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
		double coaxTarget = 0;
		switch (position) {
			case RETRACTED:
				driveTarget = RETRACTED_DRIVE;
				coaxTarget = RETRACTED_COAX;
				break;
			case PRE_INTAKE:
				driveTarget = PRE_INTAKE_DRIVE;
				coaxTarget = PRE_INTAKE_COAX;
				break;
			case INTAKE:
				driveTarget = INTAKE_DRIVE;
				coaxTarget = INTAKE_COAX;
				break;
			case TRANSFER:
			default:
				driveTarget = TRANSFER_DRIVE;
				coaxTarget = TRANSFER_COAX;
				break;
		}

		intakeDriveServoLeft.setPosition(driveTarget);
		intakeDriveServoRight.setPosition(driveTarget);
		intakeCoaxServo.setPosition(coaxTarget);
	}
	
	public void setIntakePosition(double driveTarget, double coaxTarget) {
		Intake.intakePosition = Intake.Positions.MANUAL;
		intakeDriveServoLeft.setPosition(driveTarget);
		intakeDriveServoRight.setPosition(driveTarget);
		intakeCoaxServo.setPosition(coaxTarget);
	}

	public void setSlidePosition(Positions position) {
		Intake.slidePosition = position;
		int target = 0;
		switch (position) {
			case PRE_INTAKE:
				target = SLIDE_POSITION_DEFAULT;
				break;
			case TRANSFER:
			case RETRACTED:
			default:
				target = SLIDE_TRANSFER_POSITION;
				break;
			case INTAKE:
				return;
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
	 * Rotates wrist
	 */
	public void setWristRotation(double target) {
		intakeWrist.setPosition(target);
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
					correct = (red / green >= 0.7) && (red / blue >= 0.9);
					break;
				case BLUE:
					correct = (blue / red >= 1.8) && (blue / green >= 0.8);
					break;
				case YELLOW:
					correct = (red / blue >= 1) && (green / blue >= 1.6);
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