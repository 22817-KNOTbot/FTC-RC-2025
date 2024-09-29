package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Automations {
	private static boolean automationRunning = false;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private Telemetry telemetry;

	public enum Alliance {
		RED, BLUE
	}

	public enum Basket {
		LOW, HIGH
	}

	public Automations(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;
	}

	public Automations(HardwareMap hardwareMap, boolean DEBUG, Telemetry telemetry) {
		this.hardwareMap = hardwareMap;
		this.DEBUG = DEBUG;
		this.telemetry = telemetry;
	}

	public boolean running() {
		return Automations.automationRunning;
	}

	public void setRunning(boolean automationRunning) {
		Automations.automationRunning = automationRunning;
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
		Servo wristLeftServo = hardwareMap.get(Servo.class, "wristLeftServo");
		Servo wristRightServo = hardwareMap.get(Servo.class, "wristRightServo");
		wristLeftServo.setDirection(Servo.Direction.FORWARD);
		wristLeftServo.setDirection(Servo.Direction.REVERSE);

		wristLeftServo.setPosition(deg/300);
		wristRightServo.setPosition(deg/300);
	}

	public void intake(Alliance alliance, boolean yellowAllowed) {
		automationRunning = true;
		// Hardware initializing
		Servo dropArmServo = hardwareMap.get(Servo.class, "dropArmServo");
		dropArmServo.setDirection(Servo.Direction.REVERSE);
		dropArmServo.scaleRange(0, 90/360);
		CRServo scoopServo = hardwareMap.get(CRServo.class, "scoopServo");
		scoopServo.setDirection(DcMotor.Direction.FORWARD);
		ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		// Rotate dropdown arm motor to base deg
		dropArmServo.setPosition(0);
		
		boolean samplePickedUp = false;
		while (!samplePickedUp) {
			// Start intake motor
			scoopServo.setPower(1);
			// Stop motor when color-distance sensor detects:
			//  1. Proximity (<2cm)
			while (colourRangeSensor.getDistance(DistanceUnit.MM) > 50) {
				if (DEBUG) {
					telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
					telemetry.update();
				}
			}
			//  2. Colour = yellow/our alliance
			int red = colourRangeSensor.red();
			int green = colourRangeSensor.green();
			int blue = colourRangeSensor.blue();
			if (DEBUG) {
				telemetry.addData("Color", "%d|%d|%d", red, green, blue);
				telemetry.update();
			}

			if ((yellowAllowed ? (
				(red / blue > 2.5) && (green / blue > 3)
			) : false) || (alliance == Alliance.RED ? (
				(red / green > 1.6) && (red / blue > 2)
			) : (
				(blue / red > 3.5) && (blue / green > 1.2)
			))) {
				scoopServo.setPower(0.5);
				samplePickedUp = true;
			} else {
				//   - if other alliance: reverse intake motor until proximity > 2cm
				scoopServo.setPower(0);
				while(colourRangeSensor.getDistance(DistanceUnit.MM) < 50) {
					if (DEBUG) {
						telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
						telemetry.update();
					}	
				}
			}
		}
		
		// Rotate dropdown arm motor -_ deg
		dropArmServo.setPosition(1);

		// # Transfer
		/*

		Servo wristLeftServo = hardwareMap.get(Servo.class, "wristLeftServo");
		Servo wristRightServo = hardwareMap.get(Servo.class, "wristRightServo");
		Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.scaleRange(0, 1); // TODO: Find and change this to correct value

		// wristMove range: 60-240
		wristMove(wristLeftServo, wristRightServo, 240);
		clawServo.setPosition(1);
		wristMove(wristLeftServo, wristRightServo, 60);
		*/

		// TODO: Everything

		automationRunning = false;
	}

	public void depositSample(Basket basket) {
		automationRunning = true;

		// Extend linear slide
		DcMotor slideMotor = hardwareMap.get(DcMotor.class, "motorName");
		
		// TODO: Everything
		// TODO: Find out what deposit system will be
		automationRunning = false;
	}

	public void grabSpecimen() {
		automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}

	public void hangSpecimen() {
		automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}
	
	public void ascend() {
		automationRunning = true;

		// Extend linear slides
		DcMotor slideMotor1 = hardwareMap.get(DcMotor.class, "motorName1");
		DcMotor slideMotor2 = hardwareMap.get(DcMotor.class, "motorName2");
		
		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}

	public void lowerSlides() {
		automationRunning = true;
		
		DcMotor slideMotor1 = hardwareMap.get(DcMotor.class, "motorName1");
		DcMotor slideMotor2 = hardwareMap.get(DcMotor.class, "motorName2");
		
		slideMotor1.setDirection(DcMotor.Direction.REVERSE);
		slideMotor2.setDirection(DcMotor.Direction.REVERSE);
		
		slideMotor1.setPower(1);
		slideMotor2.setPower(1);
		
		slideMotor1.setDirection(DcMotor.Direction.FORWARD);
		slideMotor2.setDirection(DcMotor.Direction.FORWARD);
		
		automationRunning = false;
	}
}

