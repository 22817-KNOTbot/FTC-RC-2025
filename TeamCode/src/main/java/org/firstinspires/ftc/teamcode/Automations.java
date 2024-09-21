package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Automations {
	private static boolean automationRunning = false;

	public boolean running() {
		return self.automationRunning;
	}

	// # Intake
	/**
	 * <h1>Intake</h1>
	 * Automation to automatically intake samples
	 * @param hardwareMap hardwareMap
	 * @param alliance true = red, false = blue
	 * @author Ethan
	 */
	public boolean intake(HardwareMap hardwareMap, boolean alliance) {
		// Dropdown arm initializing
		DcMotor dropArmMotor = hardwareMap.get(DcMotor.class, "dropArmMotor");
		dropArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		int initialRotation = dropArmMotor.getCurrentPosition();
		double ticksPerRotation = dropArmMotor.getMotorType().getTicksPerRev();

		// End of arm motor initializing
		DcMotor scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		

		// Rotate dropdown arm motor _ deg
		dropArmMotor.setTargetPosition(initalRotation + ticksPerRotation*45/360);
		
		// Start intake motor

		
		// Stop motor when color-distance sensor detects:
		//  1. Proximity (<2cm)
		//  2. Colour = yellow/our alliance
		//   - if other alliance: reverse intake motor until proximity > 2cm
		
		// Rotate dropdown arm motor -_ deg
		
		// # Transfer
		
		// TODO: Everything
	}
	
}

