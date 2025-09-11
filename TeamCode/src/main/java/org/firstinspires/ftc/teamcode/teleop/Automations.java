package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

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

		ARTIFACT_LOADED,
		ARTIFACT_EJECT_WAIT,
		ARTIFACT_EJECT,

	}

	DcMotor turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

	public void grab() {
		automationState = State.INTAKE_OPEN;
	}
	public void putIntoTurret() {
		automationState = State.TRANSFER;
	}
	public void fireTurret() {
		automationState = State.ARTIFACT_LOADED;
		turretMotor.setPower(1);
		automationState = State.ARTIFACT_EJECT;

	}

}
