package org.firstinspires.ftc.teamcode.teleop.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Automations;


public class Teleop extends LinearOpMode {

	private Automations automationHandler;
    
    @Override
	public void runOpMode() {

        waitForStart();

        while (opModeIsActive) {
			switch (automationHandler.automationState) {
				case ABORT:
					automationHandler.abort();
					break;
			}
            if (gamepad1.aWasPressed) {
                automationHandler.grab()
            }
            if (gamepad1.bWasPressed) {
                automationHandler.putIntoTurret()
            }
            if (gamepad1.xWasPressed) {
                automationHandler.fireTurret()
            }
            if (gamepad1.yWasPressed) {

            }
        }


    }

}
