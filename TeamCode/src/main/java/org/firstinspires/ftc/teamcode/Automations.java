package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Automations {
	private static boolean automationRunning = false;
	private HardwareMap hardwareMap;

	public Automations(HardwareMap hardwareMap_input) {
		hardwareMap = hardwareMap_input;
	}

	public enum Alliance {
		RED, BLUE
	}

	public enum Basket {
		LOW, HIGH
	}

	public boolean running() {
		return automationRunning;
	}

	public void intake(Alliance alliance, boolean yellowAllowed) {
		automationRunning = true;
		// Hardware initializing
		DcMotor dropArmMotor = hardwareMap.get(DcMotor.class, "dropArmMotor");
		dropArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		dropArmMotor.setPower(1);
		int initialRotation = dropArmMotor.getCurrentPosition();
		double ticksPerRotation = dropArmMotor.getMotorType().getTicksPerRev();
		
		DcMotor scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		scoopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		// scoopMotor.setDirection(DcMotor.Direction.REVERSE);

		ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		// Rotate dropdown arm motor _ deg
		dropArmMotor.setTargetPosition((int)Math.round(initialRotation + ticksPerRotation*(45/360)));
		
		// Start intake motor
		scoopMotor.setPower(1);
		colourRangeSensor.enableLed(true);
		
		boolean samplePickedUp = false;
		while (!samplePickedUp) {
			// Stop motor when color-distance sensor detects:
			//  1. Proximity (<2cm)
			while (colourRangeSensor.getDistance(DistanceUnit.MM) > 2) {}
			//  2. Colour = yellow/our alliance
			if (!(yellowAllowed ? (
				colourRangeSensor.red()>200 &&
				colourRangeSensor.green()>200 &&
				colourRangeSensor.blue()<100
			) : false) || (alliance == Alliance.RED ? (
				colourRangeSensor.red()>200 &&
				colourRangeSensor.green()<100 &&
				colourRangeSensor.blue()<100
			) : (
				colourRangeSensor.red()<100 &&
				colourRangeSensor.green()<100 &&
				colourRangeSensor.blue()>200
			))) {
			//   - if other alliance: reverse intake motor until proximity > 2cm
				while(colourRangeSensor.getDistance(DistanceUnit.MM) < 2) {
					scoopMotor.setPower(-1);
				}
				scoopMotor.setPower(0);
			}
		}

		// Rotate dropdown arm motor -_ deg
		samplePickedUp = true;
		scoopMotor.setPower(0);
		colourRangeSensor.enableLed(false);
		dropArmMotor.setTargetPosition(initialRotation);
		
		// # Transfer
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

