package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class Brakes {
	public static double LEFT_BRAKE_ENGAGED_POSITION = 0.59;
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

	public void engageBrakes(boolean engage) {
		if (engage) {
			engageBrakes();
		} else {
			releaseBrakes();
		}
	}
	
	public void engageBrakes() {
		brakeServoLeft.enablePwm();
		brakeServoRight.enablePwm();
		brakeServoLeft.setPosition(LEFT_BRAKE_ENGAGED_POSITION);
		brakeServoRight.setPosition(RIGHT_BRAKE_ENGAGED_POSITION);
		engaged = true;
	}

	public void releaseBrakes() {
		brakeServoLeft.setPosition(LEFT_BRAKE_RELEASED_POSITION);
		brakeServoRight.setPosition(RIGHT_BRAKE_RELEASED_POSITION);
		brakeServoLeft.disablePwm();
		brakeServoRight.disablePwm();
		engaged = false;
	}
}
