package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class Pto {
	public static double ptoServoLeftDisengaged = 0.0;
	public static double ptoServoLeftEngaged = 1.0;
	public static double ptoServoRightDisengaged = 0.0;
	public static double ptoServoRightEngaged = 1.0;

	private Servo ptoServoLeft;
	private Servo ptoServoRight;

	private boolean ptoEngaged = false;

	public Pto(HardwareMap hardwareMap) {
		ptoServoLeft = hardwareMap.get(Servo.class, "ptoServoLeft");
		ptoServoRight = hardwareMap.get(Servo.class, "ptoServoRight");
	}

	public void setPtoEngaged(boolean engaged) {
		ptoEngaged = engaged;
		if (engaged) {
			ptoServoLeft.setPosition(ptoServoLeftEngaged);
			ptoServoRight.setPosition(ptoServoRightEngaged);
		} else {
			ptoServoLeft.setPosition(ptoServoLeftDisengaged);
			ptoServoRight.setPosition(ptoServoRightDisengaged);
		}
	}

	public boolean getPtoEngaged() {
		return ptoEngaged;
	}
}
