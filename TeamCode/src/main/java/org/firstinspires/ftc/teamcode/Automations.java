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

	public enum Alliance {
		RED, BLUE
	}

	public enum Basket {
		LOW, HIGH
	}

	public boolean running() {
		return automationRunning;
	}

	public void setRunning(boolean running) {
		automationRunning = running;
	}

	public void wristMove(Servo leftServo, Servo rightServo, int deg) {
		leftServo.setDirection(Servo.Direction.REVERSE);

		leftServo.setPosition(deg/300);
		rightServo.setPosition(deg/300);
	}

	/*
	public void wristRotate(int deg) {
		Servo wristLeftServo = hardwareMap.get(Servo.class, "wristLeftServo");
		Servo wristRightServo = hardwareMap.get(Servo.class, "wristRightServo");
		wristLeftServo.setDirection(Servo.Direction.FORWARD);

		wristLeftServo.setPosition(deg/300);
		wristRightServo.setPosition(deg/300);
	}
	*/

	public void intake(HardwareMap hardwareMap, Alliance alliance, boolean yellowAllowed) {
		automationRunning = true;
		// Hardware initializing
		Servo dropArmServo = hardwareMap.get(Servo.class, "dropArmServo");
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
			while (colourRangeSensor.getDistance(DistanceUnit.MM) > 50) {}
			//  2. Colour = yellow/our alliance
			double red = colourRangeSensor.red();
			double green = colourRangeSensor.green();
			double blue = colourRangeSensor.blue();

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
				while(colourRangeSensor.getDistance(DistanceUnit.MM) < 50) {}
			}
		}
		
		// Rotate dropdown arm motor -_ deg
		dropArmServo.setPosition(90/360);

		// # Transfer
		
		Servo wristLeftServo = hardwareMap.get(Servo.class, "wristLeftServo");
		Servo wristRightServo = hardwareMap.get(Servo.class, "wristRightServo");
		Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.scaleRange(0, 1); // TODO: Find and change this to correct value

		// wristMove range: 60-240
		wristMove(wristLeftServo, wristRightServo, 240);
		clawServo.setPosition(1);
		wristMove(wristLeftServo, wristRightServo, 60);
		

		// TODO: Everything

		automationRunning = false;
	}

	public void depositSample(HardwareMap hardwareMap, Basket basket) {
		automationRunning = true;

		// Extend linear slide
		DcMotor slideMotor = hardwareMap.get(DcMotor.class, "motorName");
		
		// TODO: Everything
		// TODO: Find out what deposit system will be
		automationRunning = false;
	}

	public void grabSpecimen(HardwareMap hardwareMap) {
		automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}

	public void hangSpecimen(HardwareMap hardwareMap) {
		automationRunning = true;

		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}
	
	public void ascend(HardwareMap hardwareMap) {
		automationRunning = true;

		// Extend linear slides
		DcMotor slideMotor1 = hardwareMap.get(DcMotor.class, "motorName1");
		DcMotor slideMotor2 = hardwareMap.get(DcMotor.class, "motorName2");
		
		// TODO: Everything
		// TODO: Find out what system it will be
		automationRunning = false;
	}

	public void lowerSlides(HardwareMap hardwareMap) {
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

