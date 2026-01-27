package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Brakes {
	public static double LEFT_BRAKE_ENGAGED_POSITION = 0.6;
	public static double LEFT_BRAKE_RELEASED_POSITION = 0.5;
	public static double RIGHT_BRAKE_ENGAGED_POSITION = 0.4;
	public static double RIGHT_BRAKE_RELEASED_POSITION = 0.49;

	private AxonServo brakeServoLeft;
	private AxonServo brakeServoRight;

	private boolean engaged;

	public Brakes(HardwareMap hardwareMap) {
		brakeServoLeft = new AxonServo(hardwareMap.get(Servo.class, "brakeServoLeft"));
		brakeServoRight = new AxonServo(hardwareMap.get(Servo.class, "brakeServoRight"));
	}

	public void start() {
		releaseBrakes();
	}

	public boolean isBrakesEngaged() {
		return engaged;
	}

	public void engageBrakes() {
		engageBrakes(true);
	}

	public void engageBrakes(boolean engage) {
		brakeServoLeft.setPosition(LEFT_BRAKE_ENGAGED_POSITION);
		brakeServoRight.setPosition(RIGHT_BRAKE_ENGAGED_POSITION);
		engaged = true;
	}

	public void releaseBrakes() {
		brakeServoLeft.setPosition(LEFT_BRAKE_RELEASED_POSITION);
		brakeServoRight.setPosition(RIGHT_BRAKE_RELEASED_POSITION);
		engaged = false;
	}
}
