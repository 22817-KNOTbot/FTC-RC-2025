package org.firstinspires.ftc.teamcode.teleop.automations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// note to self, shift alt f == auto indent

public class Automations extends LinearOpMode {

	public State automationState;

	public enum State {
		ABORT,
		IDLE,

		// intake

		INTAKE_OPEN,
		INTAKE_OFF,

		// transfer

		TRANSFER,

		// ejection

		SAMPLE_LOADED,
		SAMPLE_EJECT_WAIT,
		SAMPLE_EJECTED,

	}

	public void grab() {
		automationState = State.INTAKE_OPEN;
	}
	public void putIntoTurret() {
		automationState = State.TRANSFER;
	}
	public void fireTurret() {
		automationState = State.SAMPLE_LOADED;
	}

}
