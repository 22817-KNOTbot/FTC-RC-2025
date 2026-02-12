package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Light {
	public static double RED = 0.28;
	public static double GREEN = 0.45;

	private Servo light;

	public Light(HardwareMap hardwareMap) {
		light = hardwareMap.get(Servo.class, "light");
	}

	public void setRed() {
		light.setPosition(RED);
	}

	public void setGreen() {
		light.setPosition(GREEN);
	}

	public void setOff() {
		light.setPosition(0);
	}
}
