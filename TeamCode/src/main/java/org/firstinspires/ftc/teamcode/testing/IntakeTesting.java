package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Configurable
// @TeleOp(name="Intake testing", group="Debug")
public class IntakeTesting extends LinearOpMode {
	public static boolean enabled = false;
	public static Float power = null;

	private Intake intake;

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		intake = new Intake(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			if (power == null) {
				intake.enable(enabled);
			} else {
				intake.setPower(power);
			}

			telemetry.addData("Enabled", enabled);
		}
	}
}