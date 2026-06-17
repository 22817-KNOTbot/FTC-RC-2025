package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Brakes;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
// @TeleOp(name="Brake testing", group="Debug")
@Disabled
public class BrakeTesting extends LinearOpMode {
	public static boolean ENGAGED = false;

	@Override
	public void runOpMode() {
		Brakes brakes = new Brakes(hardwareMap);

		waitForStart();
		
		brakes.start();

		while (opModeIsActive()) {
			brakes.engageBrakes(ENGAGED);
		}
	}
}