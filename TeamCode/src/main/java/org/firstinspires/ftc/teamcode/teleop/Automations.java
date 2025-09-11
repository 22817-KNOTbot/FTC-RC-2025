package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import com.qualcomm.robotcore.hardware.HardwareMap;

// note to self, shift alt f == auto indent

public class Automations {

	private Shooter shooter;
	public State automationState;
	private HardwareMap hardwareMap;

	public enum State {
		ABORT,
		IDLE,

		// intake

		INTAKE_OPEN,
		INTAKE_OFF,

		// transfer

		TRANSFER,

		// ejection

		ARTIFACT_LOADED,
		ARTIFACT_EJECT_WAIT,
		ARTIFACT_EJECT,

	}

	public void grab() {
		automationState = State.INTAKE_OPEN;
	}
	public void putIntoTurret() {
		automationState = State.TRANSFER;
	}
	public void fireTurret() {
		shooter = new Shooter(hardwareMap, false);

	}

}
