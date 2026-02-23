package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Configurable
public class Light {
	public static double RED = 0.28;
	public static double GREEN = 0.5;
	public static double BLUE = 0.6;

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

	public void setBlue() {
		light.setPosition(BLUE);
	}

	public void setOff() {
		light.setPosition(0);
	}
}
