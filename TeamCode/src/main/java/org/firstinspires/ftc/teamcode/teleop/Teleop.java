package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class Teleop extends LinearOpMode {

	private Automations automationHandler;
	private MecanumDrive mecanumHandler;
    
    @Override
	public void runOpMode() {

        waitForStart();

        while (opModeIsActive) {
			switch (automationHandler.automationState) {
				case ABORT:
					automationHandler.abort();
					break;
				case IDLE:
					break;
				case INTAKE_OPEN:
					break;
				case INTAKE_OFF:
					break;
				case TRANSFER:
					break;
				case ARTIFACT_LOADED:
					break;
				case ARTIFACT_EJECT_WAIT:
					break;
				case ARTIFACT_EJECTED:
					break;
			}

			mecanumHandler.mecanumDrive();

            if (gamepad1.aWasPressed) {
                automationHandler.grab();
            }
            if (gamepad1.bWasPressed) {
                automationHandler.putIntoTurret();
            }
            if (gamepad1.xWasPressed) {
                automationHandler.fireTurret();
            }
            if (gamepad1.yWasPressed) {

            }
        }


    }

}
