package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Automations {
	public static State automationState = State.IDLE;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private Telemetry telemetry;

	// Hardware mapping
	Servo wristLeftServo;
	Servo wristRightServo;
	Servo dropArmServo;
	CRServo scoopServo;
	ColorRangeSensor colourRangeSensor;
	Servo clawServo;
	// DcMotor slideMotor1;
	// DcMotor slideMotor2;

	// https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html
	public enum State {
		ABORT,
		IDLE,
		INTAKE_WAIT,
		INTAKE_FILLED,
		INTAKE_DUMPING,
		TRANSFER
	}

	public enum Alliance {
		RED, 
		BLUE
	}

	public enum Basket {
		LOW, 
		HIGH
	}

	public Automations(HardwareMap hardwareMap) {
		this(hardwareMap, false, null);
	}

	public Automations(HardwareMap hardwareMap, boolean DEBUG, Telemetry telemetry) {
		this.hardwareMap = hardwareMap;
		this.DEBUG = DEBUG;
		this.telemetry = telemetry;

		wristLeftServo = hardwareMap.get(Servo.class, "wristLeftServo");
		wristRightServo = hardwareMap.get(Servo.class, "wristRightServo");
		dropArmServo = hardwareMap.get(Servo.class, "dropArmServo");
		scoopServo = hardwareMap.get(CRServo.class, "scoopServo");
		colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		// slideMotor1 = hardwareMap.get(DcMotor.class, "motorName1");
		// slideMotor2 = hardwareMap.get(DcMotor.class, "motorName2");
	}

	public void abort() {
		// This was created if proper abort code is needed
		// Currently does nothing and just stops automations
		automationState = State.IDLE;
	}

	public void intakeInit() {
		// Hardware initializing
		dropArmServo.setDirection(Servo.Direction.REVERSE);
		dropArmServo.scaleRange(0, 90/360);
		scoopServo.setDirection(DcMotor.Direction.FORWARD);

		// Rotate dropdown arm motor to base deg
		dropArmServo.setPosition(0);
		// Start intake motor
		scoopServo.setPower(1);

		automationState = State.INTAKE_WAIT;
	}
	
	public void intakeWait() {
		// Do nothing until distance <50mm
		if (colourRangeSensor.getDistance(DistanceUnit.MM) < 50) {
			automationState = State.INTAKE_FILLED;
		}
		if (DEBUG) {
			telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
			telemetry.update();
		}
	}

	public void intakeFilled(Alliance alliance, boolean yellowAllowed) {
		// Check sample colour
		int red = colourRangeSensor.red();
		int green = colourRangeSensor.green();
		int blue = colourRangeSensor.blue();
		if (DEBUG) {
			telemetry.addData("Color", "%d|%d|%d", red, green, blue);
			telemetry.update();
		}

		if ((yellowAllowed ? (
			// Yellow check (if yellow is allowed)
			(red / blue > 2.5) && (green / blue > 3)
		) : false) || (alliance == Alliance.RED ? (
			// Red check
			(red / green > 1.6) && (red / blue > 2)
		) : (
			// Blue check
			(blue / red > 3.5) && (blue / green > 1.2)
		))) {
			// If our alliance, transfer
			scoopServo.setPower(0.5);
			automationState = State.TRANSFER;
		} else {
			// If other alliance, dump
			automationState = State.INTAKE_DUMPING;
		}
	}

	public void intakeDumping() {
		// reverse intake motor until proximity > 50mm
		scoopServo.setPower(0);
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 50) {
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
			telemetry.update();
		}

	}

	public void transfer() {
		
		// Rotate dropdown arm motor -_ deg
		dropArmServo.setPosition(1);

		// # Transfer
		/*

		clawServo.scaleRange(0, 1); // TODO: Find and change this to correct value

		// wristMove range: 60-240
		wristMove(wristLeftServo, wristRightServo, 240);
		clawServo.setPosition(1);
		wristMove(wristLeftServo, wristRightServo, 60);
		*/

		// TODO: Everything
	}

	public void depositSample(Basket basket) {
		// automationRunning = true;

		// Extend linear slide
		
		// TODO: Everything
		// TODO: Find out what deposit system will be
		automationState = State.IDLE;
	}

	public void grabSpecimen() {
		// automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationState = State.IDLE;
	}

	public void hangSpecimen() {
		// automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationState = State.IDLE;
	}
	
	public void ascend() {
		// automationRunning = true;

		// Extend linear slides
		
		// TODO: Everything
		// TODO: Find out what system it will be
		automationState = State.IDLE;
	}

	public void lowerSlides() {
		slideMotor1.setDirection(DcMotor.Direction.FORWARD);
		slideMotor2.setDirection(DcMotor.Direction.FORWARD);

		slideMotor1.setPower(-1);
		slideMotor2.setPower(-1);		
		
		automationState = State.IDLE;
	}

	public void wristMove(Servo leftServo, Servo rightServo, int deg) {
		leftServo.setDirection(Servo.Direction.REVERSE);

		leftServo.setPosition(deg/300);
		rightServo.setPosition(deg/300);

		if (DEBUG) {
			telemetry.addData("Wrist", "%.2f|.2f", leftServo.getPosition(), rightServo.getPosition());
			telemetry.update();
		}
	}

	public void wristRotate(int deg) {
		wristLeftServo.setDirection(Servo.Direction.FORWARD);
		wristLeftServo.setDirection(Servo.Direction.REVERSE);

		wristLeftServo.setPosition(deg/300);
		wristRightServo.setPosition(deg/300);
	}
}