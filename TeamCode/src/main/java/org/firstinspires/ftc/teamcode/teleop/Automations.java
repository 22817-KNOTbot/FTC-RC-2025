package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Automations {
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

	public Automations(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public Automations(HardwareMap hardwareMap, boolean DEBUG) {
		// TODO: Add subsystems after merged
	}

	public void showTelemetry(Telemetry telemetry) {
		// TODO: Add telemetry after subsystems are merged
	}

	public void grab() {
		automationState = State.INTAKE_OPEN;
	}
	public void putIntoTurret() {
		automationState = State.TRANSFER;
	}
	public void fireTurret() {

	}


	public boolean colourSensorResponding() {
		// TODO: use turret subsystem method after merged
		return true;
	}

}
