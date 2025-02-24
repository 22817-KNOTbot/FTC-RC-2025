package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Claw {
	/*
	 * Configurable constants
	 *
	 * Most positions/values can be changed here.
	 * 
	 * Directions:
	 * Lower = open
	 * Higher = closed
	 */
	public static double OPEN = 0.6;
	public static double CLOSED = 0.93;

	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private Servo clawServo;

	public static Positions position = Positions.CLOSED;

	public enum Positions {
		OPEN,
		CLOSED,
		MANUAL
	}

	public Claw(HardwareMap hardwareMap) {
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		// clawServo.setDirection(Servo.Direction.REVERSE);
	}

	/*
	 * Sets the position of the claw
	 * 
	 * There are 2 methods: enum and manual.
	 * Enum should be the primary one used, 
	 * as it also updates a variable for telemetry
	 * and allows for quick changing of positions.
	 */

	public void setPosition(Positions position) {
		Claw.position = position;
		double target = 0;
		switch (position) {
			case OPEN:
				target = OPEN;
				break;
			case CLOSED:
			default:
				target = CLOSED;
				break;
		}
		clawServo.setPosition(target);
	}

	public void setPosition(double position) {
		Claw.position = Claw.Positions.MANUAL;
		clawServo.setPosition(position);
	}
}